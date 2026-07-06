#!/usr/bin/env python3
"""Headless Ditto → Sharpa retargeting teleop (no Viser viewer).

Runs the same core loop as ``viz/view_teleop.py`` without Viser/IK on the main
thread. Ditto encoder reads still run in a background finger_aloha thread at
``hand_config.control_frequency`` (200 Hz); this process only polls the queue,
retargets, and streams to Sharpa.

Use this to check whether the viewer was starving the 200 Hz read loop (GIL
contention). Compare the periodic ``PERFORMANCE SUMMARY`` from the background
thread here vs ``view_teleop.py``.

Usage (from sharpa_teleop repo root):
    python retargeting_teleop/run_teleop.py --ditto --sharpa --3f
    python retargeting_teleop/run_teleop.py --ditto --3f --rate 200
    python retargeting_teleop/run_teleop.py --ditto --sharpa --3f --rate 200 --retarget-rate 40
    python retargeting_teleop/run_teleop.py --ditto u2d2.usb_port=/dev/ttyUSB0

``--rate`` — main-loop poll period (Hz). ``--retarget-rate`` — IK + Sharpa send
(Hz); keep lower than ``--rate`` so retargeting does not block leader polling.

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
from retargeting.paths import (  # noqa: E402
    DITTO_LEADER_ONLY_HAND_CONFIG,
    DITTO_3F_LEADER_ONLY_HAND_CONFIG,
    ditto_leader_urdf,
)
from hardware_interfaces.ditto_leader import LeaderHardwareSession  # noqa: E402
from retargeting.ditto_ik import FingerName  # noqa: E402
from retargeting.retargeter import DittoToSharpaRetargeter  # noqa: E402
from teleop.engine import RetargetTeleopEngine  # noqa: E402

# Leader polling stays fast (cheap queue drain); retargeting IK is the expensive
# part and runs decoupled at a lower rate so it does not starve the 200 Hz
# finger_aloha read thread (shared GIL).
DEFAULT_RATE_HZ = 200.0
DEFAULT_RETARGET_HZ = 40.0
LOG_INTERVAL_S = 0.5
PERF_INTERVAL_S = 5.0


@dataclass
class _MainLoopPerf:
    iterations: int = 0
    poll_ms_sum: float = 0.0
    retarget_ms_sum: float = 0.0
    loop_ms_sum: float = 0.0
    loop_ms_max: float = 0.0
    retarget_count: int = 0


@dataclass
class RunFlags:
    ditto_hardware: bool = False
    sharpa_hardware: bool = False
    ditto_3f: bool = False
    rate_hz: float = DEFAULT_RATE_HZ
    retarget_hz: float = DEFAULT_RETARGET_HZ
    perf_interval_s: float = PERF_INTERVAL_S
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
        elif arg == "--3f":
            flags.ditto_3f = True
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
        elif arg == "--perf-interval":
            flags.perf_interval_s = float(next(args))
        elif arg.startswith("--perf-interval="):
            flags.perf_interval_s = float(arg.split("=", 1)[1])
        else:
            remaining.append(arg)
    sys.argv = remaining
    return flags


def _build_overrides(cli_args: list[str], *, ditto_3f: bool = False) -> list[str]:
    """Inject finger_aloha conf searchpath (motor_models, joint_configs)."""
    searchpath = f"hydra.searchpath=[file://{FA_CONF_DIR}]"
    overrides = [searchpath]
    if not any(arg.startswith("hand_config=") for arg in cli_args):
        default_cfg = (
            DITTO_3F_LEADER_ONLY_HAND_CONFIG
            if ditto_3f
            else DITTO_LEADER_ONLY_HAND_CONFIG
        )
        overrides.append(f"hand_config={default_cfg}")
    return overrides + list(cli_args)


def _print_main_loop_perf(perf: _MainLoopPerf, *, target_hz: float) -> None:
    if perf.iterations == 0:
        return
    n = perf.iterations
    loop_avg_ms = perf.loop_ms_sum / n
    poll_avg_ms = perf.poll_ms_sum / n
    actual_hz = 1000.0 / loop_avg_ms if loop_avg_ms > 0 else 0.0
    retarget_avg = (
        perf.retarget_ms_sum / perf.retarget_count if perf.retarget_count else 0.0
    )
    print(
        f"[main loop] iter={n}  poll={poll_avg_ms:.2f}ms avg  "
        f"retarget={retarget_avg:.2f}ms avg ({perf.retarget_count}x)  "
        f"loop={loop_avg_ms:.2f}ms avg / {perf.loop_ms_max:.2f}ms max  "
        f"~{actual_hz:.0f} Hz (target {target_hz:.0f} Hz)"
    )
    perf.iterations = 0
    perf.poll_ms_sum = 0.0
    perf.retarget_ms_sum = 0.0
    perf.loop_ms_sum = 0.0
    perf.loop_ms_max = 0.0
    perf.retarget_count = 0


def main() -> None:
    flags = _strip_run_flags()
    if not (flags.ditto_hardware or flags.sharpa_hardware):
        raise SystemExit(
            "Headless teleop needs hardware: pass --ditto and/or --sharpa "
            "(use viz/view_teleop.py for a viewer-only session)."
        )

    overrides = _build_overrides(sys.argv[1:], ditto_3f=flags.ditto_3f)
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

    if flags.ditto_3f:
        fingers: tuple[FingerName, ...] = ("index", "middle", "thumb")
    elif flags.enable_thumb:
        fingers = ("index", "thumb")
    else:
        fingers = ("index",)
    engine = RetargetTeleopEngine(
        hardware=hardware,
        sharpa_follower=sharpa_follower,
        retargeter=DittoToSharpaRetargeter.from_sharpa_config(
            cfg.get("sharpa"),
            ditto_urdf=ditto_leader_urdf(kinematics_v2=True),
        ),
        fingers=fingers,
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
    control_hz = float(cfg.hand_config.get("control_frequency", 200))
    last_log = 0.0
    last_retarget = 0.0
    last_perf = time.time()
    perf = _MainLoopPerf()
    print(
        f"Headless teleop: main poll {flags.rate_hz:.0f} Hz, "
        f"retarget {flags.retarget_hz:.0f} Hz, "
        f"Ditto read thread {control_hz:.0f} Hz "
        f"(ditto={'on' if hardware else 'off'}, "
        f"sharpa={'on' if sharpa_follower else 'off'})."
    )
    if hardware is not None:
        print(
            "  Ditto reads run in a background thread — watch PERFORMANCE SUMMARY "
            "below (same metric as view_teleop.py)."
        )
    if hardware is None:
        print("  No leader hardware: holding neutral pose (Sharpa receives neutral).")

    # Stream an initial pose so the follower has a target even before leader data.
    engine.retarget(solve_params=IK_POLISH)

    try:
        while True:
            loop_t0 = time.perf_counter()
            poll_t0 = loop_t0
            has_leader = engine.poll_leader() is not None
            poll_ms = (time.perf_counter() - poll_t0) * 1000.0

            now = time.time()
            retarget_ms = 0.0
            if has_leader and now - last_retarget >= retarget_period:
                last_retarget = now
                rt0 = time.perf_counter()
                engine.retarget(solve_params=IK_STREAM)
                retarget_ms = (time.perf_counter() - rt0) * 1000.0
                perf.retarget_count += 1
                perf.retarget_ms_sum += retarget_ms

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

            loop_ms = (time.perf_counter() - loop_t0) * 1000.0
            perf.iterations += 1
            perf.poll_ms_sum += poll_ms
            perf.loop_ms_sum += loop_ms
            perf.loop_ms_max = max(perf.loop_ms_max, loop_ms)
            if now - last_perf >= flags.perf_interval_s:
                last_perf = now
                _print_main_loop_perf(perf, target_hz=flags.rate_hz)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if hardware is not None:
            hardware.stop()
        if sharpa_follower is not None:
            sharpa_follower.stop()


if __name__ == "__main__":
    main()
