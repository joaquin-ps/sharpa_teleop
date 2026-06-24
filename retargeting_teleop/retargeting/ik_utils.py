"""Shared Pinocchio IK helpers for finger pad tasks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class IkSolveParams:
    """Tunable parameters for damped least-squares pad IK."""

    max_iterations: int = 40
    step_size: float = 0.5
    damping: float = 0.05
    tolerance: float = 1e-4
    blend_gain: float = 1.0


# Few short steps + partial blend: smooth while dragging.
IK_INTERACTIVE = IkSolveParams(
    max_iterations=6,
    step_size=0.2,
    damping=0.15,
    tolerance=5e-3,
    blend_gain=0.35,
)

# Full solve used after slider edits or when a drag ends.
IK_POLISH = IkSolveParams(
    max_iterations=40,
    step_size=0.5,
    damping=0.05,
    tolerance=1e-4,
    blend_gain=1.0,
)


def joint_q_indices(model: pin.Model, joint_names: tuple[str, ...]) -> list[int]:
    return [model.joints[model.getJointId(name)].idx_q for name in joint_names]


def joint_v_indices(model: pin.Model, joint_names: tuple[str, ...]) -> list[int]:
    return [model.joints[model.getJointId(name)].idx_v for name in joint_names]


def frame_pose_in_base(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    frame_name: str,
) -> pin.SE3:
    frame_id = model.getFrameId(frame_name)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[frame_id].copy()


def scale_pad_translation_in_retarget(pad_in_retarget: pin.SE3, scale: float) -> pin.SE3:
    """Scale pad translation expressed in ``retarget_base``; orientation unchanged."""
    if scale == 1.0:
        return pad_in_retarget.copy()
    return pin.SE3(pad_in_retarget.rotation.copy(), scale * pad_in_retarget.translation)


def pad_pose_relative_to_retarget(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    *,
    retarget_base_link: str,
    pad_link: str,
) -> pin.SE3:
    """Return pad pose expressed in the hand ``retarget_base`` frame."""
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    t_retarget = data.oMf[model.getFrameId(retarget_base_link)]
    t_pad = data.oMf[model.getFrameId(pad_link)]
    return t_retarget.inverse() * t_pad


def solve_pad_ik(
    model: pin.Model,
    data: pin.Data,
    frame_name: str,
    active_joint_names: tuple[str, ...],
    q_seed: np.ndarray,
    target_in_base: pin.SE3,
    *,
    position_weight: float = 1.0,
    orientation_weight: float = 1.0,
    solve_params: IkSolveParams | None = None,
) -> tuple[np.ndarray, float]:
    """Best-effort 6D IK via damped least squares; inactive joints stay at ``q_seed``."""
    params = solve_params or IkSolveParams()
    frame_id = model.getFrameId(frame_name)
    q_idx = joint_q_indices(model, active_joint_names)
    v_idx = joint_v_indices(model, active_joint_names)
    weights = np.diag(
        [position_weight] * 3 + [orientation_weight] * 3,
    ).astype(float)

    q_seed = q_seed.copy()
    q = q_seed.copy()
    lower = np.array(
        [model.lowerPositionLimit[model.joints[model.getJointId(name)].idx_q] for name in active_joint_names]
    )
    upper = np.array(
        [model.upperPositionLimit[model.joints[model.getJointId(name)].idx_q] for name in active_joint_names]
    )

    last_error_norm = float("inf")
    for _ in range(params.max_iterations):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        error = pin.log(data.oMf[frame_id].inverse() * target_in_base).vector
        weighted_error = weights @ error
        last_error_norm = float(np.linalg.norm(weighted_error))
        if last_error_norm < params.tolerance:
            break

        jacobian = pin.computeFrameJacobian(
            model,
            data,
            q,
            frame_id,
            pin.ReferenceFrame.LOCAL,
        )
        jacobian = weights @ jacobian[:, v_idx]
        lhs = jacobian @ jacobian.T + (params.damping**2) * np.eye(6)
        joint_delta = params.step_size * jacobian.T @ np.linalg.solve(lhs, weighted_error)

        for i, (qi, delta) in enumerate(zip(q_idx, joint_delta)):
            q[qi] = float(np.clip(q[qi] + delta, lower[i], upper[i]))

    if params.blend_gain < 1.0:
        q_out = q_seed.copy()
        blend = float(np.clip(params.blend_gain, 0.0, 1.0))
        for qi in q_idx:
            q_out[qi] = (1.0 - blend) * q_seed[qi] + blend * q[qi]
        q = q_out

    return q, last_error_norm
