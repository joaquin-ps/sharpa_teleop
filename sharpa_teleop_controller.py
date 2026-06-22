#!/usr/bin/env python3
"""
Sharpa + Dynamixel teleoperation entry point.

Example:
    python sharpa_teleop/sharpa_teleop_controller.py u2d2.usb_port=/dev/ttyUSB0
    python sharpa_teleop/sharpa_teleop_controller.py \\
        hand_config.leader.joint_settings.0.current_control.enable_force_rendering=false
"""

import sys
from pathlib import Path

TELEOP_ROOT = Path(__file__).resolve().parent
REPO_ROOT = TELEOP_ROOT.parent
sys.path.insert(0, str(REPO_ROOT))

from hydra import compose, initialize_config_dir  # noqa: E402

from sharpa_teleop.sharpa_ditto_teleop import SharpaDittoTeleop  # noqa: E402

CONF_DIR = TELEOP_ROOT / "conf"
FA_CONF_DIR = (REPO_ROOT / "finger_aloha" / "hand_interfaces" / "conf").resolve()


def _build_overrides(cli_args: list[str]) -> list[str]:
    """Inject finger_aloha conf searchpath (absolute) before user overrides."""
    searchpath = f"hydra.searchpath=[file://{FA_CONF_DIR}]"
    return [searchpath] + list(cli_args)


def main() -> None:
    overrides = _build_overrides(sys.argv[1:])
    with initialize_config_dir(
        version_base=None, config_dir=str(CONF_DIR)
    ):
        cfg = compose(config_name="config", overrides=overrides)

    teleop = SharpaDittoTeleop(cfg)
    try:
        teleop.connect()
        teleop.setup_motors()
        teleop.start_control_loop()
    finally:
        teleop.disconnect()


if __name__ == "__main__":
    main()
