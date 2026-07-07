#!/usr/bin/env python3
"""Ditto leader hardware + Sharpa retargeting + Sharpa follower hardware in Viser.

Reads the physical Ditto leader and retargets Ditto→Sharpa to the Sharpa Wave
hand. Leader-only configs use ``torque_off`` (encoders only); force-rendering
configs (``leader.mode: current``) run the full haptic loop via
``RetargetForceRenderTeleop``.

Hardware is OFF by default (viewer only); opt in per interface with --ditto
and/or --sharpa.

The Sharpa pad-force arrows + would-be leader joint-torque arrows are driven by
the force source seeded from the config (hand_config.control.fingers; tactile/mix
imply --sharpa). You can switch the source live from the "Force source" dropdown
in the viewer.

Usage (from sharpa_teleop repo root):
    python retargeting_teleop/viz/view_teleop.py                 # viewer only, no hardware
    python retargeting_teleop/viz/view_teleop.py --ditto --sharpa  # both hardware
    python retargeting_teleop/viz/view_teleop.py --ditto         # ditto leader only
    python retargeting_teleop/viz/view_teleop.py --sharpa        # sharpa hand only
    python retargeting_teleop/viz/view_teleop.py --ditto --sharpa hand_config=ditto_hand_tactile
    python retargeting_teleop/viz/view_teleop.py --ditto u2d2.fake_u2d2=true
    python retargeting_teleop/viz/view_teleop.py --ditto --3f       # 3-finger Ditto hardware + URDF
    python retargeting_teleop/viz/view_teleop.py --ditto u2d2.usb_port=/dev/ttyUSB0

From retargeting_teleop/ package dir:
    python viz/view_teleop.py --ditto --sharpa
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

_PKG = Path(__file__).resolve().parent.parent
_REPO = _PKG.parent
for _path in (_PKG, _REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from hydra import compose, initialize_config_dir  # noqa: E402

from _paths import CONF_DIR, DITTO_CONF_DIR  # noqa: E402
from retargeting.paths import DITTO_LEADER_ONLY_HAND_CONFIG, DITTO_3F_LEADER_ONLY_HAND_CONFIG  # noqa: E402
from hardware_interfaces.ditto_leader import LeaderHardwareSession  # noqa: E402
from teleop.force_render import (  # noqa: E402
    RetargetForceRenderTeleop,
    config_needs_tactile,
    is_force_render_config,
    viewer_initial_source,
)
from viz.view_assets import run_viewer  # noqa: E402


@dataclass
class ViewFlags:
    show_leader: bool = True
    show_sharpa: bool = True
    ditto_3f: bool = False
    ditto_hardware: bool = False
    sharpa_hardware: bool = False


def _strip_view_flags() -> ViewFlags:
    """Remove viewer flags before Hydra parses ``sys.argv``.

    Hardware is opt-in: ``--ditto`` enables the Ditto leader, ``--sharpa``
    enables the Sharpa follower. ``--leader-only`` / ``--sharpa-only`` only
    control which URDFs are shown. The control mode is config-only
    (``hand_config.control.fingers``); tactile/mix auto-enable ``--sharpa``.
    """
    flags = ViewFlags()
    remaining = [sys.argv[0]]
    for arg in sys.argv[1:]:
        if arg == "--leader-only":
            flags.show_sharpa = False
        elif arg == "--sharpa-only":
            flags.show_leader = False
        elif arg == "--ditto":
            flags.ditto_hardware = True
        elif arg == "--sharpa":
            flags.sharpa_hardware = True
        elif arg == "--3f":
            flags.ditto_3f = True
        else:
            remaining.append(arg)
    sys.argv = remaining

    # Hardware needs the corresponding hand(s) in the scene; retargeting to the
    # Sharpa hand requires both the Ditto leader and Sharpa hands shown.
    flags.ditto_hardware = flags.ditto_hardware and flags.show_leader
    flags.sharpa_hardware = (
        flags.sharpa_hardware and flags.show_leader and flags.show_sharpa
    )
    return flags


def _build_overrides(cli_args: list[str], *, ditto_3f: bool = False) -> list[str]:
    """Inject ditto conf searchpath (motor_models, joint_configs)."""
    searchpath = f"hydra.searchpath=[file://{DITTO_CONF_DIR}]"
    overrides = [searchpath]
    if not any(arg.startswith("hand_config=") for arg in cli_args):
        default_cfg = (
            DITTO_3F_LEADER_ONLY_HAND_CONFIG
            if ditto_3f
            else DITTO_LEADER_ONLY_HAND_CONFIG
        )
        overrides.append(f"hand_config={default_cfg}")
    return overrides + list(cli_args)


def main() -> None:
    flags = _strip_view_flags()
    overrides = _build_overrides(sys.argv[1:], ditto_3f=flags.ditto_3f)

    with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
        cfg = compose(config_name="config", overrides=overrides)

    # Force source + tactile options are config-only. The viewer uses one global
    # source (with a live dropdown); seed it from the config's per-finger modes.
    force_mode = viewer_initial_source(cfg)
    fr_cfg = cfg.hand_config.get("force_render") or {}
    tactile_calibrate = bool(fr_cfg.get("calibrate", False))
    tactile_debug = bool(fr_cfg.get("debug", False))
    # tactile/mix/measured need Sharpa → auto-enable when both hands shown.
    if config_needs_tactile(cfg) and flags.show_leader and flags.show_sharpa:
        flags.sharpa_hardware = True
    if is_force_render_config(cfg) and flags.ditto_hardware:
        flags.sharpa_hardware = flags.sharpa_hardware or flags.show_sharpa

    hardware = None
    force_render_controller: RetargetForceRenderTeleop | None = None
    sharpa_follower = None
    if flags.sharpa_hardware:
        from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

        sharpa_follower = SharpaFollowerSession(cfg, verbose=True)

    if flags.ditto_hardware:
        if is_force_render_config(cfg):
            print("Connecting Ditto leader (current mode / force rendering)...")
            force_render_controller = RetargetForceRenderTeleop(
                cfg, sharpa_follower=sharpa_follower
            )
            force_render_controller.connect()
            force_render_controller.setup_motors()
            threading.Thread(
                target=force_render_controller.start_control_loop,
                daemon=True,
                name="ditto-force-render",
            ).start()
            time.sleep(0.3)
            print(
                f"Leader motors: {list(cfg.hand_config.leader.motor_ids)} "
                f"(mode={cfg.hand_config.leader.mode}, haptics on)"
            )
            # Sharpa lifecycle is owned by the force-render controller.
            sharpa_follower = None
        else:
            hardware = LeaderHardwareSession(cfg)
            print("Connecting Ditto leader hardware...")
            hardware.start()
            print(
                f"Leader motors: {list(cfg.hand_config.leader.motor_ids)} "
                f"(mode={cfg.hand_config.leader.mode}, "
                f"follower={cfg.hand_config.follower.mode})"
            )

    try:
        run_viewer(
            show_leader=flags.show_leader,
            show_sharpa=flags.show_sharpa,
            ditto_3f=flags.ditto_3f,
            hardware=hardware,
            sharpa_follower=sharpa_follower,
            force_render_controller=force_render_controller,
            force_mode=force_mode,
            tactile_calibrate=tactile_calibrate,
            tactile_debug=tactile_debug,
            sharpa_cfg=cfg.sharpa,
        )
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if force_render_controller is not None:
            force_render_controller.stop()
            force_render_controller.disconnect()
        if hardware is not None:
            hardware.stop()
        if sharpa_follower is not None:
            sharpa_follower.stop()


if __name__ == "__main__":
    main()
