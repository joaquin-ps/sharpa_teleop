"""Map retargeted Sharpa URDF joint angles → Sharpa SDK joint targets.

The IK retargeter (``retargeting/``) works in the Sharpa *URDF* convention. The
Sharpa Wave SDK (``sharpa_controller/``) uses its own joint names and zero/sign
convention. This module bridges the two and is where to calibrate per-joint
sign and offset once the physical hand is connected.

This module has no Sharpa SDK dependency, so it is safe to import anywhere.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

# Sharpa URDF joint name (retargeting) -> Sharpa SDK joint name (sharpa_controller).
SHARPA_URDF_TO_SDK_JOINT: dict[str, str] = {
    "right_index_MCP_FE": "Index MCP Flexion/Extension",
    "right_index_MCP_AA": "Index MCP Abduction/Adduction",
    "right_index_PIP": "Index PIP Flexion/Extension",
    "right_index_DIP": "Index DIP Flexion/Extension",
    "right_thumb_CMC_FE": "Thumb CMC Flexion/Extension",
    "right_thumb_CMC_AA": "Thumb CMC Abduction/Adduction",
    "right_thumb_MCP_FE": "Thumb MCP Flexion/Extension",
    "right_thumb_MCP_AA": "Thumb MCP Abduction/Adduction",
    "right_thumb_IP": "Thumb DIP Flexion/Extension",
}

# Per-joint URDF -> SDK correction (identity until calibrated on hardware):
#   sdk_angle = sign * urdf_q + offset_rad
SHARPA_URDF_TO_SDK_SIGN: dict[str, float] = {
    name: 1.0 for name in SHARPA_URDF_TO_SDK_JOINT
}
SHARPA_URDF_TO_SDK_OFFSET_RAD: dict[str, float] = {
    name: 0.0 for name in SHARPA_URDF_TO_SDK_JOINT
}

# SDK joint names driven by the follower (enabled for position control).
SHARPA_FOLLOWER_SDK_JOINTS: tuple[str, ...] = tuple(SHARPA_URDF_TO_SDK_JOINT.values())

# Global sign relating SDK-reported joint torque to the external contact wrench.
# The SDK reports the actuator's holding torque, which opposes the applied force,
# so we flip it so the estimated pad force points *against* the external push.
# (This is separate from SHARPA_URDF_TO_SDK_SIGN, which fixes joint-axis direction.)
SHARPA_TORQUE_FEEDBACK_SIGN: float = -1.0


def urdf_q_to_sdk_targets(
    sharpa_q: np.ndarray,
    q_index_of: Callable[[str], int],
) -> dict[str, float]:
    """Convert a Sharpa URDF configuration vector to SDK joint targets (rad).

    ``q_index_of`` maps a URDF joint name to its index in ``sharpa_q`` (e.g.
    ``SharpaFingerIK.joint_q_index``).
    """
    targets: dict[str, float] = {}
    for urdf_name, sdk_name in SHARPA_URDF_TO_SDK_JOINT.items():
        angle = float(sharpa_q[q_index_of(urdf_name)])
        targets[sdk_name] = (
            SHARPA_URDF_TO_SDK_SIGN[urdf_name] * angle
            + SHARPA_URDF_TO_SDK_OFFSET_RAD[urdf_name]
        )
    return targets
