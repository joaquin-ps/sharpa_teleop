#!/usr/bin/env python3
"""Ditto leader hardware + Sharpa retargeting in Viser.

Reads the physical Ditto leader via finger_aloha (leader-only, torque_off, no
force feedback) and drives the bundled URDF viewer + Ditto→Sharpa retargeting.

Usage (from sharpa_teleop repo root):
    python retargeting_teleop/viz/view_teleop.py
    python retargeting_teleop/viz/view_teleop.py hand_config=ditto_7dof_leader_only
    python retargeting_teleop/viz/view_teleop.py u2d2.fake_u2d2=true
    python retargeting_teleop/viz/view_teleop.py --leader-only
    python retargeting_teleop/viz/view_teleop.py u2d2.usb_port=/dev/ttyUSB0

From retargeting_teleop/ package dir:
    python viz/view_teleop.py
"""

from __future__ import annotations

import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parent.parent
_REPO = _PKG.parent
for _path in (_PKG, _REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from hydra import compose, initialize_config_dir  # noqa: E402

from _paths import CONF_DIR, FA_CONF_DIR  # noqa: E402
from retargeting.paths import DITTO_LEADER_ONLY_HAND_CONFIG  # noqa: E402
from hardware_interfaces.ditto_leader import LeaderHardwareSession  # noqa: E402
from viz.view_assets import run_viewer  # noqa: E402


def _strip_view_flags() -> tuple[bool, bool]:
    """Remove viewer flags before Hydra parses ``sys.argv``."""
    show_leader = True
    show_sharpa = True
    remaining = [sys.argv[0]]
    for arg in sys.argv[1:]:
        if arg == "--leader-only":
            show_sharpa = False
        elif arg == "--sharpa-only":
            show_leader = False
        else:
            remaining.append(arg)
    sys.argv = remaining
    return show_leader, show_sharpa


def _build_overrides(cli_args: list[str]) -> list[str]:
    """Inject finger_aloha conf searchpath (motor_models, joint_configs)."""
    searchpath = f"hydra.searchpath=[file://{FA_CONF_DIR}]"
    overrides = [searchpath]
    if not any(arg.startswith("hand_config=") for arg in cli_args):
        overrides.append(f"hand_config={DITTO_LEADER_ONLY_HAND_CONFIG}")
    return overrides + list(cli_args)


def main() -> None:
    show_leader, show_sharpa = _strip_view_flags()
    overrides = _build_overrides(sys.argv[1:])

    with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
        cfg = compose(config_name="config", overrides=overrides)

    hardware = LeaderHardwareSession(cfg)
    print("Connecting Ditto leader hardware...")
    hardware.start()
    print(
        f"Leader motors: {list(cfg.hand_config.leader.motor_ids)} "
        f"(mode={cfg.hand_config.leader.mode}, follower={cfg.hand_config.follower.mode})"
    )
    try:
        run_viewer(
            show_leader=show_leader,
            show_sharpa=show_sharpa,
            hardware=hardware,
        )
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        hardware.stop()


if __name__ == "__main__":
    main()
