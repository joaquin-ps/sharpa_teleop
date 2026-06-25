"""Viser-free core loop for Ditto → Sharpa retargeting teleop.

``RetargetTeleopEngine`` owns the kinematic retargeting plus references to the
optional hardware sessions, and exposes the steady-state operations both the GUI
viewer and the headless runner need:

- ``poll_leader()``        read Ditto leader hardware → leader ``q``
- ``retarget()``           Ditto ``q`` → Sharpa ``q`` (and stream to follower)
- ``estimate_force_feedback()``  read follower torques → estimated pad forces and
  would-be leader joint torques (observe-only)

It carries no Viser dependency and holds session *references* only; lifecycle
(start/stop) stays with the caller, which has the Hydra config and console UX.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import pinocchio as pin

from retargeting.ditto_ik import FingerName
from retargeting.ik_utils import IkSolveParams
from retargeting.paths import DITTO_LEADER_JOINT_NAMES
from retargeting.retargeter import DittoToSharpaRetargeter, RetargetResult

if TYPE_CHECKING:
    from hardware_interfaces.ditto_leader.session import LeaderHardwareSession
    from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

_FINGERS: tuple[FingerName, ...] = ("index", "thumb")


@dataclass
class ForceFeedbackSample:
    """One observe-only force-feedback estimate (no Viser, no hardware command)."""

    q_meas: np.ndarray  # measured Sharpa configuration (URDF / pin order)
    sharpa_forces: dict[str, tuple[np.ndarray, np.ndarray]]  # finger → (origin, F) Sharpa base
    sharpa_force_vec: dict[str, np.ndarray]  # finger → F in Sharpa base
    magnitudes: dict[str, float]  # finger → |F| (N)
    leader_forces: dict[str, tuple[np.ndarray, np.ndarray]]  # finger → (origin, F) leader base
    leader_torques: dict[str, float]  # leader joint name → would-be torque (Nm)
    torque_arrows: list[tuple[np.ndarray, np.ndarray, float]]  # (origin, axis, tau) leader base


class RetargetTeleopEngine:
    """Hardware-aware, Viser-free core for Ditto → Sharpa retargeting teleop."""

    def __init__(
        self,
        *,
        hardware: "LeaderHardwareSession | None" = None,
        sharpa_follower: "SharpaFollowerSession | None" = None,
        retargeter: DittoToSharpaRetargeter | None = None,
    ) -> None:
        self.retargeter = retargeter if retargeter is not None else DittoToSharpaRetargeter()
        self.hardware = hardware
        self.sharpa_follower = sharpa_follower
        self.sharpa_send_enabled = sharpa_follower is not None
        self.ditto_q = pin.neutral(self.ditto_ik.model)
        self.sharpa_q = pin.neutral(self.sharpa_ik.model)

    @property
    def ditto_ik(self):
        return self.retargeter.ditto

    @property
    def sharpa_ik(self):
        return self.retargeter.sharpa

    def ditto_q_from_actuated(
        self,
        angles: np.ndarray,
        joint_names: tuple[str, ...] = DITTO_LEADER_JOINT_NAMES,
    ) -> np.ndarray:
        """Build a full leader pin ``q`` from actuated joint angles (URDF order)."""
        q = np.zeros(self.ditto_ik.model.nq, dtype=float)
        for name, value in zip(joint_names, np.asarray(angles, dtype=float)):
            q[self.ditto_ik.joint_q_index(name)] = float(value)
        return q

    def poll_leader(self) -> np.ndarray | None:
        """Read leader hardware → updated ``ditto_q`` (or ``None`` if no new data)."""
        if self.hardware is None:
            return None
        angles = self.hardware.poll_joint_angles()
        if angles is None:
            return None
        self.ditto_q = self.ditto_q_from_actuated(angles)
        return self.ditto_q

    def retarget(
        self,
        ditto_q: np.ndarray | None = None,
        sharpa_q_seed: np.ndarray | None = None,
        *,
        solve_params: IkSolveParams | None = None,
        send: bool = True,
    ) -> RetargetResult:
        """Retarget leader → Sharpa and (optionally) stream to follower hardware.

        ``ditto_q`` / ``sharpa_q_seed`` default to the engine's stored state, so
        the headless loop can call ``retarget()`` with no arguments. The GUI
        passes both explicitly (its source of truth is the Viser sliders).
        """
        if ditto_q is not None:
            self.ditto_q = np.asarray(ditto_q, dtype=float)
        seed = self.sharpa_q if sharpa_q_seed is None else np.asarray(sharpa_q_seed, dtype=float)
        result = self.retargeter.retarget(self.ditto_q, seed, solve_params=solve_params)
        self.sharpa_q = result.sharpa_q
        if (
            send
            and self.sharpa_follower is not None
            and self.sharpa_send_enabled
        ):
            self.sharpa_follower.send_q(result.sharpa_q)
        return result

    def estimate_force_feedback(
        self,
        sharpa_q_seed: np.ndarray | None = None,
        ditto_q: np.ndarray | None = None,
    ) -> ForceFeedbackSample | None:
        """Read follower torques → estimated pad forces + would-be leader torques.

        Returns ``None`` when no follower is attached or no measurement is
        available yet. Nothing is commanded to leader hardware.
        """
        if self.sharpa_follower is None:
            return None
        seed = self.sharpa_q if sharpa_q_seed is None else np.asarray(sharpa_q_seed, dtype=float)
        leader_q = self.ditto_q if ditto_q is None else np.asarray(ditto_q, dtype=float)

        inputs = self.sharpa_follower.read_wrench_inputs(seed)
        if inputs is None:
            return None
        q_meas, finger_torques = inputs

        sharpa_forces: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        sharpa_force_vec: dict[str, np.ndarray] = {}
        magnitudes: dict[str, float] = {}
        for finger in _FINGERS:
            force = self.sharpa_ik.estimate_pad_force(q_meas, finger, finger_torques[finger])
            origin = np.asarray(
                self.sharpa_ik.pad_pose_in_base(q_meas, finger).translation, dtype=float
            )
            sharpa_forces[finger] = (origin, force)
            sharpa_force_vec[finger] = force
            magnitudes[finger] = float(np.linalg.norm(force))

        leader_forces: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        leader_torques: dict[str, float] = {}
        torque_arrows: list[tuple[np.ndarray, np.ndarray, float]] = []
        for finger in _FINGERS:
            fb = self.retargeter.leader_force_and_torque(
                finger, sharpa_force_vec[finger], q_meas, leader_q
            )
            leader_forces[finger] = (
                fb.pad_origin_in_leader_base,
                fb.force_in_leader_base,
            )
            for name, tau, origin, axis in zip(
                fb.joint_names, fb.joint_torques, fb.joint_origins, fb.joint_axes
            ):
                leader_torques[name] = float(tau)
                torque_arrows.append((origin, axis, float(tau)))

        return ForceFeedbackSample(
            q_meas=q_meas,
            sharpa_forces=sharpa_forces,
            sharpa_force_vec=sharpa_force_vec,
            magnitudes=magnitudes,
            leader_forces=leader_forces,
            leader_torques=leader_torques,
            torque_arrows=torque_arrows,
        )
