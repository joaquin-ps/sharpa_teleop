#!/usr/bin/env python3
"""Headless retargeting force rendering for the Ditto leader.

Drives the configured Ditto leader motors in CURRENT mode and renders a haptic
force synthesized from the estimated Ditto joint torque (Jᵀ·F from the Sharpa
contact force). The Sharpa hand is streamed the retargeted pose via the follower
session. Which finger(s) are retargeted/rendered is inferred from the leader
motors in the selected ``hand_config``.

The control mode is fully defined by the chosen ``hand_config``: each finger's
position and force source are declared together under ``control.fingers``
(position = retarget | joint; force = measured | estimate | tactile | mix).
There are no force-source CLI flags — pick or edit a config instead. A per-finger
summary is printed at startup. Configs (``conf/hand_config/``):
    ditto_index_force_render   index: retarget + estimate
    ditto_thumb_force_render   thumb: retarget + estimate
    ditto_index_tactile        index: retarget + tactile
    ditto_middle_tactile       middle: retarget + tactile
    ditto_thumb_tactile        thumb: retarget + tactile
    ditto_hand_tactile         index+thumb: retarget + tactile
    ditto_3f_tactile           index+middle+thumb: retarget + tactile
    ditto_hand_mixed           index: joint + measured;  thumb: retarget + tactile
    ditto_hand_mixed_ik        index: retarget + measured; thumb: retarget + tactile

Tactile/mix sources auto-enable the Sharpa follower; set
``force_render.calibrate: true`` / ``force_render.debug: true`` in the config to
calibrate tactile on startup / print raw F6 vs base force.

Examples:
    # run a config as-is (tactile config auto-enables Sharpa):
    python retargeting_teleop/run_force_render.py hand_config=ditto_hand_tactile
    python retargeting_teleop/run_force_render.py hand_config=ditto_3f_tactile
    python retargeting_teleop/run_force_render.py hand_config=ditto_middle_tactile
    # explicitly add the Sharpa hand for an estimate config:
    python retargeting_teleop/run_force_render.py hand_config=ditto_index_force_render --sharpa
    # raise a per-joint gain at runtime:
    python retargeting_teleop/run_force_render.py hand_config=ditto_hand_tactile \
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
from teleop.force_render import (  # noqa: E402
    RetargetForceRenderTeleop,
    config_needs_tactile,
)

DEFAULT_HAND_CONFIG = "ditto_index_force_render"
DEFAULT_RETARGET_HZ = 40.0


@dataclass
class RunFlags:
    sharpa_hardware: bool = False
    retarget_hz: float = DEFAULT_RETARGET_HZ


def _strip_run_flags() -> RunFlags:
    flags = RunFlags()
    remaining = [sys.argv[0]]
    args = iter(sys.argv[1:])
    for arg in args:
        if arg == "--sharpa":
            flags.sharpa_hardware = True
        elif arg == "--retarget-rate":
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

    # Control mode is config-only (hand_config.control.fingers). Tactile/mix
    # sources need real Sharpa contact data, so they auto-enable the follower.
    need_sharpa = flags.sharpa_hardware or config_needs_tactile(cfg)

    sharpa_follower = None
    if need_sharpa:
        # Imported lazily: this pulls in the Sharpa Wave SDK.
        from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

        sharpa_follower = SharpaFollowerSession(cfg, verbose=True)

    controller = RetargetForceRenderTeleop(
        cfg,
        sharpa_follower=sharpa_follower,
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
