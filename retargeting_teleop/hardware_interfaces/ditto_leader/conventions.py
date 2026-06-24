"""Hardware encoder convention → Ditto URDF joint angles."""

from __future__ import annotations

import numpy as np

from retargeting.paths import DITTO_INDEX_JOINT_NAMES, DITTO_LEADER_JOINT_NAMES

# Per-joint sign: hardware encoder + vs URDF + (index joints are negated).
DITTO_LEADER_HARDWARE_JOINT_SIGNS: tuple[float, ...] = tuple(
    -1.0 if name in DITTO_INDEX_JOINT_NAMES else 1.0 for name in DITTO_LEADER_JOINT_NAMES
)

_SIGNS = np.array(DITTO_LEADER_HARDWARE_JOINT_SIGNS, dtype=float)


def hardware_joint_angles_to_urdf(angles: np.ndarray) -> np.ndarray:
    """Map finger_aloha leader joint angles (rad) to Ditto URDF ``q`` convention."""
    q = np.asarray(angles, dtype=float)
    if q.shape[0] != len(DITTO_LEADER_JOINT_NAMES):
        raise ValueError(
            f"Expected {len(DITTO_LEADER_JOINT_NAMES)} joint angles, got {q.shape[0]}"
        )
    return q * _SIGNS
