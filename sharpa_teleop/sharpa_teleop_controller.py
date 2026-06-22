#!/usr/bin/env python3
"""
Sharpa + Dynamixel teleoperation entry point.

Example (from repo root):
    python sharpa_teleop/sharpa_teleop_controller.py u2d2.usb_port=/dev/ttyUSB0

Example (from sharpa_teleop/ package dir):
    python sharpa_teleop_controller.py \\
        hand_config.leader.joint_settings.0.current_control.enable_force_rendering=false
"""

import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from hydra import compose, initialize_config_dir  # noqa: E402

from sharpa_teleop._paths import CONF_DIR, FA_CONF_DIR  # noqa: E402
from sharpa_teleop.sharpa_ditto_teleop import SharpaDittoTeleop  # noqa: E402


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
