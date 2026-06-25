"""Ditto leader → Sharpa right-hand kinematic retargeting (no Viser dependency)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from .ditto_ik import DittoFingerIK, FingerName
from .ik_utils import (
    IkSolveParams,
    frame_pose_in_base,
    pad_pose_relative_to_retarget,
    scale_pad_translation_in_retarget,
)
from .paths import (
    DITTO_FINGERTIP_LINKS,
    DITTO_LEADER_URDF,
    DITTO_RETARGET_BASE_LINK,
    SHARPA_RETARGET_BASE_LINK,
    SHARPA_RIGHT_URDF,
)
from .sharpa_ik import SharpaFingerIK

_DITTO_PADS: dict[FingerName, str] = {
    "index": DITTO_FINGERTIP_LINKS[0],
    "thumb": DITTO_FINGERTIP_LINKS[1],
}


@dataclass(frozen=True)
class LeaderForceFeedback:
    """Sharpa pad force mapped onto the Ditto leader for one finger."""

    force_in_leader_base: np.ndarray  # 3D force at the leader pad (leader base axes)
    pad_origin_in_leader_base: np.ndarray  # leader pad origin (leader base frame)
    joint_torques: np.ndarray  # would-be leader joint torques (Nm)
    joint_names: tuple[str, ...]  # leader joints, same order as joint_torques
    joint_origins: np.ndarray  # (n,3) joint origins in leader base frame
    joint_axes: np.ndarray  # (n,3) unit rotation axes in leader base frame


@dataclass(frozen=True)
class RetargetResult:
    """Output of one Ditto → Sharpa retargeting step."""

    sharpa_q: np.ndarray
    index_residual: float
    thumb_residual: float
    index_pad_in_retarget: pin.SE3
    thumb_pad_in_retarget: pin.SE3
    index_target_in_sharpa_base: pin.SE3
    thumb_target_in_sharpa_base: pin.SE3
    index_achieved_in_sharpa_base: pin.SE3
    thumb_achieved_in_sharpa_base: pin.SE3


class DittoToSharpaRetargeter:
    """Map Ditto index/thumb pads to Sharpa pads in each hand's ``retarget_base`` frame."""

    def __init__(
        self,
        *,
        ditto_urdf: Path = DITTO_LEADER_URDF,
        sharpa_urdf: Path = SHARPA_RIGHT_URDF,
        index_position_weight: float = 1.5,
        index_orientation_weight: float = 0.1,
        thumb_position_weight: float = 1.5,
        thumb_orientation_weight: float = 0.05,
        index_cartesian_scale: float = 1.3,
        thumb_cartesian_scale: float = 1.3,
    ) -> None:
        self.ditto = DittoFingerIK(ditto_urdf)
        self.sharpa = SharpaFingerIK(sharpa_urdf)
        self.index_position_weight = index_position_weight
        self.index_orientation_weight = index_orientation_weight
        self.thumb_position_weight = thumb_position_weight
        self.thumb_orientation_weight = thumb_orientation_weight
        self.index_cartesian_scale = index_cartesian_scale
        self.thumb_cartesian_scale = thumb_cartesian_scale

    def ditto_pad_relative_to_retarget(
        self,
        ditto_q: np.ndarray,
        finger: FingerName,
    ) -> pin.SE3:
        return pad_pose_relative_to_retarget(
            self.ditto.model,
            self.ditto.data,
            ditto_q,
            retarget_base_link=DITTO_RETARGET_BASE_LINK,
            pad_link=_DITTO_PADS[finger],
        )

    def sharpa_pad_target_in_base(
        self,
        sharpa_q_seed: np.ndarray,
        pad_in_retarget: pin.SE3,
    ) -> pin.SE3:
        t_retarget = frame_pose_in_base(
            self.sharpa.model,
            self.sharpa.data,
            sharpa_q_seed,
            SHARPA_RETARGET_BASE_LINK,
        )
        return t_retarget * pad_in_retarget

    def _scaled_pad_in_retarget(
        self,
        pad_in_retarget: pin.SE3,
        finger: FingerName,
    ) -> pin.SE3:
        scale = (
            self.index_cartesian_scale
            if finger == "index"
            else self.thumb_cartesian_scale
        )
        return scale_pad_translation_in_retarget(pad_in_retarget, scale)

    def leader_force_and_torque(
        self,
        finger: FingerName,
        sharpa_force_in_sharpa_base: np.ndarray,
        sharpa_q: np.ndarray,
        ditto_q: np.ndarray,
    ) -> LeaderForceFeedback:
        """Map an estimated Sharpa pad force onto the Ditto leader (no rendering).

        The two hands correspond through their ``retarget_base`` frames, so the
        force is re-expressed there, scaled for power-consistency under the
        per-finger cartesian scale (``F_leader = scale · F_sharpa``), rotated into
        the leader base frame, and projected onto leader joints via ``Jᵀ``.
        """
        scale = (
            self.index_cartesian_scale
            if finger == "index"
            else self.thumb_cartesian_scale
        )
        r_sharpa_rb = frame_pose_in_base(
            self.sharpa.model, self.sharpa.data, sharpa_q, SHARPA_RETARGET_BASE_LINK
        ).rotation
        force_in_retarget = r_sharpa_rb.T @ np.asarray(
            sharpa_force_in_sharpa_base, dtype=float
        )

        r_ditto_rb = frame_pose_in_base(
            self.ditto.model, self.ditto.data, ditto_q, DITTO_RETARGET_BASE_LINK
        ).rotation
        force_in_leader_base = scale * (r_ditto_rb @ force_in_retarget)

        j_lin = self.ditto.pad_jacobian(ditto_q, finger)[:3, :]
        joint_torques = j_lin.T @ force_in_leader_base
        pad_origin = self.ditto.pad_pose_in_base(ditto_q, finger).translation

        joint_frames = self.ditto.finger_joint_frames_in_base(ditto_q, finger)
        joint_origins = np.asarray([o for o, _ in joint_frames], dtype=float)
        joint_axes = np.asarray([a for _, a in joint_frames], dtype=float)

        return LeaderForceFeedback(
            force_in_leader_base=force_in_leader_base,
            pad_origin_in_leader_base=np.asarray(pad_origin, dtype=float),
            joint_torques=joint_torques,
            joint_names=self.ditto.finger_joint_names(finger),
            joint_origins=joint_origins,
            joint_axes=joint_axes,
        )

    def retarget(
        self,
        ditto_q: np.ndarray,
        sharpa_q_seed: np.ndarray,
        *,
        solve_params: IkSolveParams | None = None,
    ) -> RetargetResult:
        """Compute Sharpa ``q`` so index/thumb pads match Ditto relative to ``retarget_base``."""
        index_in_retarget = self.ditto_pad_relative_to_retarget(ditto_q, "index")
        thumb_in_retarget = self.ditto_pad_relative_to_retarget(ditto_q, "thumb")
        index_scaled = self._scaled_pad_in_retarget(index_in_retarget, "index")
        thumb_scaled = self._scaled_pad_in_retarget(thumb_in_retarget, "thumb")

        q = sharpa_q_seed.copy()
        index_target = self.sharpa_pad_target_in_base(q, index_scaled)
        q, index_residual = self.sharpa.solve_finger_pad(
            "index",
            index_target,
            q,
            position_weight=self.index_position_weight,
            orientation_weight=self.index_orientation_weight,
            solve_params=solve_params,
        )

        thumb_target = self.sharpa_pad_target_in_base(q, thumb_scaled)
        q, thumb_residual = self.sharpa.solve_finger_pad(
            "thumb",
            thumb_target,
            q,
            position_weight=self.thumb_position_weight,
            orientation_weight=self.thumb_orientation_weight,
            solve_params=solve_params,
        )

        index_achieved = self.sharpa.pad_pose_in_base(q, "index")
        thumb_achieved = self.sharpa.pad_pose_in_base(q, "thumb")

        return RetargetResult(
            sharpa_q=q,
            index_residual=index_residual,
            thumb_residual=thumb_residual,
            index_pad_in_retarget=index_in_retarget,
            thumb_pad_in_retarget=thumb_in_retarget,
            index_target_in_sharpa_base=index_target,
            thumb_target_in_sharpa_base=thumb_target,
            index_achieved_in_sharpa_base=index_achieved,
            thumb_achieved_in_sharpa_base=thumb_achieved,
        )
