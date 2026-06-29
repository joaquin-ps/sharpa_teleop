#!/usr/bin/env python3
"""Headless Ditto → Sharpa retargeting teleop (no Viser viewer).

Runs the same core loop as the GUI (``viz/view_teleop.py``) without a browser:
read the physical Ditto leader, retarget Ditto → Sharpa, stream to the Sharpa
Wave follower, and (observe-only) log estimated pad forces + would-be leader
joint torques.

Hardware is opt-in per interface with ``--ditto`` and/or ``--sharpa`` (at least
one is required, since there is nothing to visualize headlessly).

Usage (from sharpa_teleop repo root):
    python retargeting_teleop/run_teleop.py --ditto --sharpa
    python retargeting_teleop/run_teleop.py --ditto u2d2.usb_port=/dev/ttyUSB0
    python retargeting_teleop/run_teleop.py --ditto --rate 200 --retarget-rate 40

``--rate`` is the (cheap) leader-poll/force-loop rate; ``--retarget-rate`` is the
(expensive) IK rate, kept lower so it does not starve the 200 Hz finger_aloha
read thread.

From retargeting_teleop/ package dir:
    python run_teleop.py --ditto --sharpa
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path

_PKG = Path(__file__).resolve().parent
_REPO = _PKG.parent
for _path in (_PKG, _REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from hydra import compose, initialize_config_dir  # noqa: E402

from _paths import CONF_DIR, FA_CONF_DIR  # noqa: E402
from retargeting.ik_utils import IK_POLISH, IK_STREAM  # noqa: E402
from retargeting.paths import DITTO_LEADER_ONLY_HAND_CONFIG  # noqa: E402
from hardware_interfaces.ditto_leader import LeaderHardwareSession  # noqa: E402
from teleop.engine import RetargetTeleopEngine  # noqa: E402

# Leader polling stays fast (cheap queue drain); retargeting IK is the expensive
# part and runs decoupled at a lower rate so it does not starve the 200 Hz
# finger_aloha read thread (shared GIL).
DEFAULT_RATE_HZ = 100.0
DEFAULT_RETARGET_HZ = 40.0
LOG_INTERVAL_S = 0.5


@dataclass
class RunFlags:
    ditto_hardware: bool = False
    sharpa_hardware: bool = False
    rate_hz: float = DEFAULT_RATE_HZ
    retarget_hz: float = DEFAULT_RETARGET_HZ
    enable_thumb: bool = True


def _strip_run_flags() -> RunFlags:
    """Remove runner flags before Hydra parses ``sys.argv``."""
    flags = RunFlags()
    remaining = [sys.argv[0]]
    args = iter(sys.argv[1:])
    for arg in args:
        if arg == "--ditto":
            flags.ditto_hardware = True
        elif arg == "--sharpa":
            flags.sharpa_hardware = True
        elif arg == "--no-thumb":
            flags.enable_thumb = False
        elif arg == "--rate":
            flags.rate_hz = float(next(args))
        elif arg.startswith("--rate="):
            flags.rate_hz = float(arg.split("=", 1)[1])
        elif arg == "--retarget-rate":
            flags.retarget_hz = float(next(args))
        elif arg.startswith("--retarget-rate="):
            flags.retarget_hz = float(arg.split("=", 1)[1])
        else:
            remaining.append(arg)
    sys.argv = remaining
    return flags


def _build_overrides(cli_args: list[str]) -> list[str]:
    """Inject finger_aloha conf searchpath (motor_models, joint_configs)."""
    searchpath = f"hydra.searchpath=[file://{FA_CONF_DIR}]"
    overrides = [searchpath]
    if not any(arg.startswith("hand_config=") for arg in cli_args):
        overrides.append(f"hand_config={DITTO_LEADER_ONLY_HAND_CONFIG}")
    return overrides + list(cli_args)


def main() -> None:
    flags = _strip_run_flags()
    if not (flags.ditto_hardware or flags.sharpa_hardware):
        raise SystemExit(
            "Headless teleop needs hardware: pass --ditto and/or --sharpa "
            "(use viz/view_teleop.py for a viewer-only session)."
        )

    overrides = _build_overrides(sys.argv[1:])
    with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
        cfg = compose(config_name="config", overrides=overrides)

    hardware = None
    if flags.ditto_hardware:
        hardware = LeaderHardwareSession(cfg)
        print("Connecting Ditto leader hardware...")
        hardware.start()
        print(
            f"Leader motors: {list(cfg.hand_config.leader.motor_ids)} "
            f"(mode={cfg.hand_config.leader.mode}, "
            f"follower={cfg.hand_config.follower.mode})"
        )

    sharpa_follower = None
    if flags.sharpa_hardware:
        # Imported lazily: this pulls in the Sharpa Wave SDK.
        from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

        sharpa_follower = SharpaFollowerSession(cfg, verbose=True)

    fingers = ("index", "thumb") if flags.enable_thumb else ("index",)
    engine = RetargetTeleopEngine(
        hardware=hardware, sharpa_follower=sharpa_follower, fingers=fingers
    )

    if sharpa_follower is not None:
        print("Connecting Sharpa follower hardware...")
        try:
            sharpa_follower.start(engine.sharpa_ik.joint_q_index)
        except Exception as exc:  # noqa: BLE001
            print(f"  Sharpa follower unavailable, continuing without it: {exc}")
            sharpa_follower = None
            engine.sharpa_follower = None

    period = 1.0 / flags.rate_hz
    retarget_period = 1.0 / flags.retarget_hz
    last_log = 0.0
    last_retarget = 0.0
    print(
        f"Headless retargeting teleop: poll {flags.rate_hz:.0f} Hz, "
        f"retarget {flags.retarget_hz:.0f} Hz "
        f"(ditto={'on' if hardware else 'off'}, "
        f"sharpa={'on' if sharpa_follower else 'off'}). Ctrl+C to exit."
    )
    if hardware is None:
        print("  No leader hardware: holding neutral pose (Sharpa receives neutral).")

    # Stream an initial pose so the follower has a target even before leader data.
    engine.retarget(solve_params=IK_POLISH)

    try:
        while True:
            # Poll fast (cheap), but only run the expensive IK at retarget_hz so
            # the high-rate finger_aloha read thread keeps its cadence.
            has_leader = engine.poll_leader() is not None
            now = time.time()
            if has_leader and now - last_retarget >= retarget_period:
                last_retarget = now
                engine.retarget(solve_params=IK_STREAM)

            sample = engine.estimate_force_feedback()
            if sample is not None and now - last_log >= LOG_INTERVAL_S:
                last_log = now
                mags = "  ".join(
                    f"{f} |F|={m:.2f}N" for f, m in sample.magnitudes.items()
                )
                taus = ", ".join(
                    f"{n} {t:+.3f}" for n, t in sample.leader_torques.items()
                )
                print(f"[force] {mags}\n  leader tau(Nm): {taus}")

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if hardware is not None:
            hardware.stop()
        if sharpa_follower is not None:
            sharpa_follower.stop()


if __name__ == "__main__":
    main()
