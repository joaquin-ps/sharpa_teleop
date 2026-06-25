"""Inverse kinematics for Sharpa index / thumb retargeting pads."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import numpy as np
import pinocchio as pin

from .ik_utils import (
    IkSolveParams,
    frame_pose_in_base,
    joint_v_indices,
    solve_pad_ik,
)
from .paths import (
    SHARPA_INDEX_JOINT_NAMES,
    SHARPA_RETARGETING_PAD_LINKS,
    SHARPA_THUMB_JOINT_NAMES,
)

FingerName = Literal["index", "thumb"]

_FINGER_TO_PAD: dict[FingerName, str] = {
    "index": SHARPA_RETARGETING_PAD_LINKS[0],
    "thumb": SHARPA_RETARGETING_PAD_LINKS[1],
}

_FINGER_TO_JOINTS: dict[FingerName, tuple[str, ...]] = {
    "index": SHARPA_INDEX_JOINT_NAMES,
    "thumb": SHARPA_THUMB_JOINT_NAMES,
}


class SharpaFingerIK:
    """Best-effort 6D pad IK on a per-finger subset of Sharpa joints."""

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

    def finger_joint_names(self, finger: FingerName) -> tuple[str, ...]:
        return _FINGER_TO_JOINTS[finger]

    def pad_jacobian(self, q: np.ndarray, finger: FingerName) -> np.ndarray:
        """6×n pad Jacobian (base-aligned axes at the pad) for the finger joints."""
        frame_id = self.model.getFrameId(_FINGER_TO_PAD[finger])
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        jacobian = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        v_idx = joint_v_indices(self.model, _FINGER_TO_JOINTS[finger])
        return jacobian[:, v_idx]

    def estimate_pad_force(
        self,
        q: np.ndarray,
        finger: FingerName,
        finger_joint_torques: np.ndarray,
        *,
        damping: float = 1e-3,
    ) -> np.ndarray:
        """Least-squares 3D pad force [fx,fy,fz] from finger joint torques.

        Models the contact as a pure force at the pad (no moment) and uses the
        linear block ``J_v`` (3×n) of the pad Jacobian. Solves ``tau = J_vᵀ F``
        for ``F`` via damped least squares (``F = (J_v J_vᵀ + λ²I)⁻¹ J_v tau``);
        overdetermined for both fingers (n>3). ``F`` is in base-aligned axes at
        the pad origin. ``finger_joint_torques`` must be ordered like
        ``finger_joint_names(finger)``.
        """
        j_lin = self.pad_jacobian(q, finger)[:3, :]
        tau = np.asarray(finger_joint_torques, dtype=float)
        lhs = j_lin @ j_lin.T + (damping**2) * np.eye(3)
        rhs = j_lin @ tau
        return np.linalg.solve(lhs, rhs)

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
        """Solve IK for one retargeting pad; other joints stay at ``q_seed``."""
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
