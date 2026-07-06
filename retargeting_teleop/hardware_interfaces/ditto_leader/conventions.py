"""Hardware encoder convention → Ditto URDF joint angles."""

from __future__ import annotations

import numpy as np

from retargeting.paths import (
    DITTO_3F_LEADER_JOINT_NAMES,
    DITTO_3F_LEADER_MOTOR_IDS,
    DITTO_INDEX_JOINT_NAMES,
    DITTO_LEADER_JOINT_NAMES,
    DITTO_LEADER_MOTOR_IDS,
)

# Per-joint sign: hardware encoder + vs URDF +.
# Canonical leader kinematics use ditto_3f_leader_v2 (+Z joint axes): no flip.
def ditto_hardware_joint_signs(
    joint_names: tuple[str, ...],
) -> tuple[float, ...]:
    return tuple(1.0 for _ in joint_names)


DITTO_LEADER_HARDWARE_JOINT_SIGNS: tuple[float, ...] = ditto_hardware_joint_signs(
    DITTO_LEADER_JOINT_NAMES
)
DITTO_3F_LEADER_HARDWARE_JOINT_SIGNS: tuple[float, ...] = ditto_hardware_joint_signs(
    DITTO_3F_LEADER_JOINT_NAMES
)


_MOTOR_TO_JOINT_NAME = dict(
    zip(DITTO_3F_LEADER_MOTOR_IDS, DITTO_3F_LEADER_JOINT_NAMES, strict=True)
)


def leader_joint_names_for_motor_ids(
    motor_ids: list[int] | tuple[int, ...],
) -> tuple[str, ...]:
    """Map a leader motor chain to Ditto URDF joint names (hardware order)."""
    motors = list(motor_ids)
    if motors == list(DITTO_3F_LEADER_MOTOR_IDS):
        return DITTO_3F_LEADER_JOINT_NAMES
    if motors == list(DITTO_LEADER_MOTOR_IDS):
        return DITTO_LEADER_JOINT_NAMES
    try:
        return tuple(_MOTOR_TO_JOINT_NAME[motor] for motor in motors)
    except KeyError as exc:
        raise ValueError(
            "Leader motor_ids must be a known Ditto leader chain or a subset of "
            f"10-DoF motors {list(DITTO_3F_LEADER_MOTOR_IDS)}, got {motors}"
        ) from exc


def hardware_joint_angles_to_urdf(
    angles: np.ndarray,
    joint_names: tuple[str, ...] = DITTO_LEADER_JOINT_NAMES,
) -> np.ndarray:
    """Map finger_aloha leader joint angles (rad) to Ditto URDF ``q`` convention."""
    q = np.asarray(angles, dtype=float)
    if q.shape[0] != len(joint_names):
        raise ValueError(
            f"Expected {len(joint_names)} joint angles, got {q.shape[0]}"
        )
    signs = np.array(ditto_hardware_joint_signs(joint_names), dtype=float)
    return q * signs
