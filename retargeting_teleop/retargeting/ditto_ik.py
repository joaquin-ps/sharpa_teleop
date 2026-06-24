"""Inverse kinematics for the Ditto leader fingerpads (Pinocchio + scipy)."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import numpy as np
import pinocchio as pin

from .ik_utils import IkSolveParams, frame_pose_in_base, solve_pad_ik
from .paths import (
    DITTO_FINGERTIP_LINKS,
    DITTO_INDEX_JOINT_NAMES,
    DITTO_THUMB_JOINT_NAMES,
)

FingerName = Literal["index", "thumb"]

_FINGER_TO_PAD: dict[FingerName, str] = {
    "index": DITTO_FINGERTIP_LINKS[0],
    "thumb": DITTO_FINGERTIP_LINKS[1],
}

_FINGER_TO_JOINTS: dict[FingerName, tuple[str, ...]] = {
    "index": DITTO_INDEX_JOINT_NAMES,
    "thumb": DITTO_THUMB_JOINT_NAMES,
}


class DittoFingerIK:
    """Best-effort 6D pad IK on a per-finger subset of Ditto leader joints."""

    def __init__(self, urdf_path: Path) -> None:
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self._joint_q_index = {
            name: self.model.joints[self.model.getJointId(name)].idx_q
            for name in self.model.names[1:]
        }

    def joint_q_index(self, joint_name: str) -> int:
        return self._joint_q_index[joint_name]

    def pad_pose_in_base(self, q: np.ndarray, finger: FingerName) -> pin.SE3:
        return frame_pose_in_base(self.model, self.data, q, _FINGER_TO_PAD[finger])

    def solve_finger_pad(
        self,
        finger: FingerName,
        target_in_base: pin.SE3,
        q_seed: np.ndarray,
        *,
        position_weight: float = 1.0,
        orientation_weight: float = 1.0,
        solve_params: IkSolveParams | None = None,
    ) -> tuple[np.ndarray, float]:
        """Solve IK for one fingerpad; other joints stay at ``q_seed``."""
        return solve_pad_ik(
            self.model,
            self.data,
            _FINGER_TO_PAD[finger],
            _FINGER_TO_JOINTS[finger],
            q_seed,
            target_in_base,
            position_weight=position_weight,
            orientation_weight=orientation_weight,
            solve_params=solve_params,
        )
