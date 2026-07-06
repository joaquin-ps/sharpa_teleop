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
    DITTO_LEADER_URDF,
    DITTO_RETARGET_BASE_LINK,
    SHARPA_RETARGET_BASE_LINK,
    SHARPA_RIGHT_URDF,
    mirror_sharpa_middle_to_ring_pinky,
    parse_ring_pinky_mirror_offsets,
    RingPinkyMirrorOffsets,
)
from .sharpa_ik import SharpaFingerIK

_DITTO_PADS: dict[FingerName, str] = {
    "index": "index_fingerpad",
    "middle": "middle_fingerpad",
    "thumb": "thumb_fingerpad",
}

_RETARGET_FINGER_ORDER: tuple[FingerName, ...] = ("index", "middle", "thumb")


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
    middle_residual: float
    thumb_residual: float
    index_pad_in_retarget: pin.SE3
    middle_pad_in_retarget: pin.SE3
    thumb_pad_in_retarget: pin.SE3
    index_target_in_sharpa_base: pin.SE3
    middle_target_in_sharpa_base: pin.SE3
    thumb_target_in_sharpa_base: pin.SE3
    index_achieved_in_sharpa_base: pin.SE3
    middle_achieved_in_sharpa_base: pin.SE3
    thumb_achieved_in_sharpa_base: pin.SE3


class DittoToSharpaRetargeter:
    """Map Ditto finger pads to Sharpa pads in each hand's ``retarget_base`` frame."""

    def __init__(
        self,
        *,
        ditto_urdf: Path = DITTO_LEADER_URDF,
        sharpa_urdf: Path = SHARPA_RIGHT_URDF,
        index_position_weight: float = 1.5,
        index_orientation_weight: float = 0.1,
        middle_position_weight: float = 1.5,
        middle_orientation_weight: float = 0.1,
        thumb_position_weight: float = 1.5,
        thumb_orientation_weight: float = 0.1,
        index_cartesian_scale: float = 1.3,
        middle_cartesian_scale: float = 1.3,
        thumb_cartesian_scale: float = 1.2,
        ring_pinky_mirror_offset_rad: RingPinkyMirrorOffsets | None = None,
    ) -> None:
        self.ditto = DittoFingerIK(ditto_urdf)
        self.sharpa = SharpaFingerIK(sharpa_urdf)
        self.index_position_weight = index_position_weight
        self.index_orientation_weight = index_orientation_weight
        self.middle_position_weight = middle_position_weight
        self.middle_orientation_weight = middle_orientation_weight
        self.thumb_position_weight = thumb_position_weight
        self.thumb_orientation_weight = thumb_orientation_weight
        self.index_cartesian_scale = index_cartesian_scale
        self.middle_cartesian_scale = middle_cartesian_scale
        self.thumb_cartesian_scale = thumb_cartesian_scale
        self.ring_pinky_mirror_offset_rad = parse_ring_pinky_mirror_offsets(
            ring_pinky_mirror_offset_rad
        )

    @classmethod
    def from_sharpa_config(
        cls,
        sharpa_cfg,
        *,
        ditto_urdf: Path = DITTO_LEADER_URDF,
        sharpa_urdf: Path = SHARPA_RIGHT_URDF,
        **kwargs,
    ) -> "DittoToSharpaRetargeter":
        """Build a retargeter, reading ring/pinky mirror offsets from Hydra config."""
        offsets = parse_ring_pinky_mirror_offsets(
            sharpa_cfg.get("ring_pinky_mirror_offset_rad")
            if sharpa_cfg is not None
            else None
        )
        return cls(
            ditto_urdf=ditto_urdf,
            sharpa_urdf=sharpa_urdf,
            ring_pinky_mirror_offset_rad=offsets,
            **kwargs,
        )

    def _ditto_has_finger(self, finger: FingerName) -> bool:
        return self.ditto.model.existFrame(_DITTO_PADS[finger])

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
        scale = {
            "index": self.index_cartesian_scale,
            "middle": self.middle_cartesian_scale,
            "thumb": self.thumb_cartesian_scale,
        }[finger]
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
        scale = {
            "index": self.index_cartesian_scale,
            "middle": self.middle_cartesian_scale,
            "thumb": self.thumb_cartesian_scale,
        }[finger]
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

    def _finger_weights(self, finger: FingerName) -> tuple[float, float]:
        if finger == "index":
            return self.index_position_weight, self.index_orientation_weight
        if finger == "middle":
            return self.middle_position_weight, self.middle_orientation_weight
        return self.thumb_position_weight, self.thumb_orientation_weight

    def retarget(
        self,
        ditto_q: np.ndarray,
        sharpa_q_seed: np.ndarray,
        *,
        solve_params: IkSolveParams | None = None,
        fingers: tuple[FingerName, ...] = ("index", "thumb"),
    ) -> RetargetResult:
        """Compute Sharpa ``q`` so the requested pads match Ditto relative to ``retarget_base``.

        Disabled fingers (not in ``fingers``) keep their seed configuration; their
        target/achieved poses report the seed pad pose and a zero residual.
        """
        q = sharpa_q_seed.copy()
        in_retarget: dict[FingerName, pin.SE3] = {}
        target: dict[FingerName, pin.SE3] = {}
        achieved: dict[FingerName, pin.SE3] = {}
        residual: dict[FingerName, float] = {}

        for finger in _RETARGET_FINGER_ORDER:
            if not self._ditto_has_finger(finger):
                target[finger] = self.sharpa.pad_pose_in_base(q, finger)
                achieved[finger] = target[finger]
                in_retarget[finger] = pin.SE3.Identity()
                residual[finger] = 0.0
                continue

            in_retarget[finger] = self.ditto_pad_relative_to_retarget(ditto_q, finger)
            if finger in fingers:
                scaled = self._scaled_pad_in_retarget(in_retarget[finger], finger)
                target[finger] = self.sharpa_pad_target_in_base(q, scaled)
                pos_w, ori_w = self._finger_weights(finger)
                q, residual[finger] = self.sharpa.solve_finger_pad(
                    finger,
                    target[finger],
                    q,
                    position_weight=pos_w,
                    orientation_weight=ori_w,
                    solve_params=solve_params,
                )
            else:
                target[finger] = self.sharpa.pad_pose_in_base(q, finger)
                residual[finger] = 0.0
            achieved[finger] = self.sharpa.pad_pose_in_base(q, finger)

        mirror_sharpa_middle_to_ring_pinky(
            q, self.sharpa.joint_q_index, self.ring_pinky_mirror_offset_rad
        )

        return RetargetResult(
            sharpa_q=q,
            index_residual=residual["index"],
            middle_residual=residual["middle"],
            thumb_residual=residual["thumb"],
            index_pad_in_retarget=in_retarget["index"],
            middle_pad_in_retarget=in_retarget["middle"],
            thumb_pad_in_retarget=in_retarget["thumb"],
            index_target_in_sharpa_base=target["index"],
            middle_target_in_sharpa_base=target["middle"],
            thumb_target_in_sharpa_base=target["thumb"],
            index_achieved_in_sharpa_base=achieved["index"],
            middle_achieved_in_sharpa_base=achieved["middle"],
            thumb_achieved_in_sharpa_base=achieved["thumb"],
        )
