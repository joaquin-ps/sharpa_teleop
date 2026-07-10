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

from retargeting.paths import RingPinkyMirrorOffsets, ring_pinky_mirrored_angle

# Sharpa URDF joint name (retargeting) -> Sharpa SDK joint name (sharpa_controller).
SHARPA_URDF_TO_SDK_JOINT: dict[str, str] = {
    "right_index_MCP_FE": "Index MCP Flexion/Extension",
    "right_index_MCP_AA": "Index MCP Abduction/Adduction",
    "right_index_PIP": "Index PIP Flexion/Extension",
    "right_index_DIP": "Index DIP Flexion/Extension",
    "right_middle_MCP_FE": "Middle MCP Flexion/Extension",
    "right_middle_MCP_AA": "Middle MCP Abduction/Adduction",
    "right_middle_PIP": "Middle PIP Flexion/Extension",
    "right_middle_DIP": "Middle DIP Flexion/Extension",
    "right_thumb_CMC_FE": "Thumb CMC Flexion/Extension",
    "right_thumb_CMC_AA": "Thumb CMC Abduction/Adduction",
    "right_thumb_MCP_FE": "Thumb MCP Flexion/Extension",
    "right_thumb_MCP_AA": "Thumb MCP Abduction/Adduction",
    "right_thumb_IP": "Thumb DIP Flexion/Extension",
}

# Ring / pinky hardware: copy the corresponding middle SDK joint target.
SHARPA_MIDDLE_MIRROR_SDK_JOINTS: dict[str, str] = {
    "Ring MCP Flexion/Extension": "Middle MCP Flexion/Extension",
    "Ring MCP Abduction/Adduction": "Middle MCP Abduction/Adduction",
    "Ring PIP Flexion/Extension": "Middle PIP Flexion/Extension",
    "Ring DIP Flexion/Extension": "Middle DIP Flexion/Extension",
    "Pinky MCP Flexion/Extension": "Middle MCP Flexion/Extension",
    "Pinky MCP Abduction/Adduction": "Middle MCP Abduction/Adduction",
    "Pinky PIP Flexion/Extension": "Middle PIP Flexion/Extension",
    "Pinky DIP Flexion/Extension": "Middle DIP Flexion/Extension",
}

# Middle SDK joint name -> mirror offset suffix (see ring_pinky_mirrored_angle).
SHARPA_MIDDLE_SDK_MIRROR_SUFFIX: dict[str, str] = {
    "Middle MCP Flexion/Extension": "MCP_FE",
    "Middle MCP Abduction/Adduction": "MCP_AA",
    "Middle PIP Flexion/Extension": "PIP",
    "Middle DIP Flexion/Extension": "DIP",
}

# SDK joints with no retargeted URDF source (held fixed on hardware).
SHARPA_FOLLOWER_FIXED_SDK_TARGETS: dict[str, float] = {
    "Pinky CMC Flexion/Extension": 0.0,
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
SHARPA_FOLLOWER_SDK_JOINTS: tuple[str, ...] = tuple(
    dict.fromkeys(
        (
            *SHARPA_URDF_TO_SDK_JOINT.values(),
            *SHARPA_MIDDLE_MIRROR_SDK_JOINTS,
            *SHARPA_FOLLOWER_FIXED_SDK_TARGETS,
        )
    )
)

# Global sign relating SDK-reported joint torque to the external contact wrench.
# The SDK reports the actuator's holding torque, which opposes the applied force,
# so we flip it so the estimated pad force points *against* the external push.
# (This is separate from SHARPA_URDF_TO_SDK_SIGN, which fixes joint-axis direction.)
SHARPA_TORQUE_FEEDBACK_SIGN: float = -1.0

# --- Fingertip tactile sensors -------------------------------------------------
# Tactile channel per finger. Right hand uses channels 0..4 (pinky, ring, middle,
# index, thumb); left hand uses 5..9. The retargeting URDF is the right hand.
SHARPA_TACTILE_CHANNEL_RIGHT: dict[str, int] = {
    "index": 3,
    "middle": 2,
    "thumb": 4,
}

# Sign relating the tactile F6 force to the same "points against the push"
# convention as the torque estimate. With the sensor->link rotation corrected, the
# raw F6 already points the right way, so this is +1. Flip to -1 if tactile-driven
# feedback comes out opposite again.
SHARPA_TACTILE_FORCE_SIGN: float = 1.0

# URDF link each fingertip tactile sensor is physically mounted on. The F6 force
# triplet is expressed (up to the residual rotation below) in THIS link's frame,
# NOT the Ditto-matched ``*_retargeting_pad`` frame. We FK this link to rotate the
# force into Sharpa base axes (matching the torque-estimate source output).
SHARPA_TACTILE_SENSOR_LINK_RIGHT: dict[str, str] = {
    "index": "right_index_fingertip",
    "middle": "right_middle_fingertip",
    "thumb": "right_thumb_fingertip",
}

# Residual fixed rotation (rpy, rad) from the tactile sensor hardware frame to its
# mounting-link frame (``SHARPA_TACTILE_SENSOR_LINK_RIGHT``). The F6 axis order
# does not match the fingertip-link axes: empirically a sensor press cycles as
# (x,y,z) -> (y,z,x) in base, which this rotation cancels.
#   R_link_sensor = [[0,1,0],[0,0,1],[1,0,0]]  (sensor x->link z, y->x, z->y)
# Same hardware/link convention on every finger, so index and thumb share it. To
# re-calibrate: press straight onto a pad and compare the tactile base-force arrow
# against the torque-estimate arrow (already correct in base axes); set
# ``force_render.debug: true`` in the hand_config to print raw F6 vs base force.
_TACTILE_SENSOR_TO_LINK_RPY = (-1.5707963267948966, 0.0, 1.5707963267948966)
SHARPA_TACTILE_SENSOR_TO_LINK_RPY: dict[str, tuple[float, float, float]] = {
    "index": _TACTILE_SENSOR_TO_LINK_RPY,
    "middle": _TACTILE_SENSOR_TO_LINK_RPY,
    "thumb": _TACTILE_SENSOR_TO_LINK_RPY,
}


def urdf_q_to_sdk_targets(
    sharpa_q: np.ndarray,
    q_index_of: Callable[[str], int],
    ring_pinky_mirror_offsets: RingPinkyMirrorOffsets | None = None,
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
    for dst, src in SHARPA_MIDDLE_MIRROR_SDK_JOINTS.items():
        suffix = SHARPA_MIDDLE_SDK_MIRROR_SUFFIX[src]
        finger = "ring" if dst.startswith("Ring ") else "pinky"
        targets[dst] = ring_pinky_mirrored_angle(
            targets[src], suffix, finger, ring_pinky_mirror_offsets
        )
    targets.update(SHARPA_FOLLOWER_FIXED_SDK_TARGETS)
    return targets
