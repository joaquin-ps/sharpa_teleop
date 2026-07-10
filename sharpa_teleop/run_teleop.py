#!/usr/bin/env python3
"""Headless Ditto–Sharpa teleop with force rendering.

Drives the configured Ditto leader motors in CURRENT mode and renders a haptic
force synthesized from Sharpa contact (tactile / estimate / measured / blend).
The Sharpa follower is always connected. Which finger(s) are driven/rendered is
inferred from the leader motors in the selected ``hand_config``.

The control mode is fully defined by the chosen ``hand_config``: each finger's
position and force source are declared together under ``control.fingers``
(position = retarget | joint; force = measured | estimate | tactile | weight dict).
A per-finger summary is printed at startup. Top-level configs (``conf/hand_config/``):
    ditto_2f_tactile          index+thumb: retarget + tactile
    ditto_3f_tactile          index+middle+thumb: retarget + tactile
    ditto_2f_blend            index joint+blend, thumb retarget+blend
    ditto_3f_blend            index/middle joint+blend, thumb retarget+blend
    joint/sharpa_3dof_index   index: joint map + measured force
    joint/sharpa_3dof_middle  middle: joint map + measured force

Set ``force_render.calibrate: true`` / ``force_render.debug: true`` in the config
to calibrate tactile on startup / print raw F6 vs base force.

Examples:
    python sharpa_teleop/run_teleop.py hand_config=ditto_2f_tactile
    python sharpa_teleop/run_teleop.py hand_config=ditto_3f_blend
    python sharpa_teleop/run_teleop.py hand_config=joint/sharpa_3dof_index
    python sharpa_teleop/run_teleop.py hand_config=ditto_2f_tactile \
        hand_config.leader.joint_settings.1.current_control.force_rendering_gain=0.05

Use ``u2d2.fake_u2d2=true`` to exercise the leader path without USB hardware.
Run ``viz/force_plot.py`` in a second terminal for the live diagnostic plot.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

_PKG = Path(__file__).resolve().parent
_REPO = _PKG.parent
for _path in (_PKG, _REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from hydra import compose, initialize_config_dir  # noqa: E402

from _paths import CONF_DIR, DITTO_CONF_DIR  # noqa: E402
from teleop.ditto_sharpa_teleop import DittoSharpaTeleop  # noqa: E402

DEFAULT_HAND_CONFIG = "ditto_2f_tactile"
DEFAULT_RETARGET_HZ = 40.0


@dataclass
class RunFlags:
    retarget_hz: float = DEFAULT_RETARGET_HZ


def _strip_run_flags() -> RunFlags:
    flags = RunFlags()
    remaining = [sys.argv[0]]
    args = iter(sys.argv[1:])
    for arg in args:
        if arg == "--retarget-rate":
            flags.retarget_hz = float(next(args))
        elif arg.startswith("--retarget-rate="):
            flags.retarget_hz = float(arg.split("=", 1)[1])
        else:
            remaining.append(arg)
    sys.argv = remaining
    return flags


def _build_overrides(cli_args: list[str]) -> list[str]:
    searchpath = f"hydra.searchpath=[file://{DITTO_CONF_DIR}]"
    overrides = [searchpath]
    if not any(arg.startswith("hand_config=") for arg in cli_args):
        overrides.append(f"hand_config={DEFAULT_HAND_CONFIG}")
    return overrides + list(cli_args)


def main() -> None:
    flags = _strip_run_flags()
    overrides = _build_overrides(sys.argv[1:])
    with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
        cfg = compose(config_name="config", overrides=overrides)

    from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

    controller = DittoSharpaTeleop(
        cfg,
        sharpa_follower=SharpaFollowerSession(cfg, verbose=True),
        retarget_hz=flags.retarget_hz,
    )

    try:
        controller.connect()
        controller.setup_motors()
        controller.start_control_loop()
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
