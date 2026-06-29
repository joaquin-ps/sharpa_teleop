"""Pluggable Sharpa pad-force sources for force feedback.

A *force source* answers a single question for the teleop engine: given the
Sharpa follower and a seed configuration, what is the contact force on each
finger pad (expressed in the Sharpa base frame), and what is the measured Sharpa
configuration?

    read(follower, q_seed_full) -> (q_meas_full, {finger: F_in_sharpa_base}) | None

Two interchangeable sources are provided, plus a blend:

- ``TorqueEstimateForceSource`` — model-based: solve ``tau = Jᵀ F`` from the SDK
  joint torques (the original behavior).
- ``TactileForceSource`` — sensor-based: read the fingertip tactile F6 wrench and
  rotate its force into the Sharpa base frame.
- ``MixedForceSource`` — weighted blend of any sources (estimate ⊕ tactile).

All sources output force in the **same frame** (Sharpa base axes at the pad), so
they are drop-in interchangeable in ``RetargetTeleopEngine.force_source``.

This module is Viser-free and SDK-free; it only duck-types the follower
(``read_wrench_inputs`` and, for tactile, ``read_tactile_f6``).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

import numpy as np
import pinocchio as pin

from hardware_interfaces.sharpa_follower.conventions import (
    SHARPA_TACTILE_FORCE_SIGN,
    SHARPA_TACTILE_SENSOR_LINK_RIGHT,
    SHARPA_TACTILE_SENSOR_TO_LINK_RPY,
)
from retargeting.ditto_ik import FingerName
from retargeting.sharpa_ik import SharpaFingerIK

if TYPE_CHECKING:
    from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

# (q_meas_full, {finger: force in Sharpa base axes at the pad})
ForceRead = tuple[np.ndarray, dict[str, np.ndarray]]


@runtime_checkable
class PadForceSource(Protocol):
    """Produces per-finger pad force (Sharpa base axes) from the follower."""

    name: str

    def read(
        self, follower: "SharpaFollowerSession", q_seed_full: np.ndarray
    ) -> ForceRead | None: ...


class TorqueEstimateForceSource:
    """Model-based pad force: damped least squares ``F`` from joint torques."""

    name = "estimate"

    def __init__(
        self,
        sharpa_ik: SharpaFingerIK,
        fingers: tuple[FingerName, ...],
        *,
        damping: float = 1e-3,
    ) -> None:
        self.sharpa_ik = sharpa_ik
        self.fingers = tuple(fingers)
        self.damping = damping

    def read(
        self, follower: "SharpaFollowerSession", q_seed_full: np.ndarray
    ) -> ForceRead | None:
        inputs = follower.read_wrench_inputs(q_seed_full)
        if inputs is None:
            return None
        q_meas, finger_torques = inputs
        forces: dict[str, np.ndarray] = {}
        for finger in self.fingers:
            tau = finger_torques.get(finger)
            if tau is None:
                continue
            forces[finger] = self.sharpa_ik.estimate_pad_force(
                q_meas, finger, tau, damping=self.damping
            )
        return q_meas, forces


class TactileForceSource:
    """Sensor-based pad force: fingertip tactile F6 rotated into Sharpa base axes.

    The F6 force triplet is expressed in the tactile sensor frame, which sits on
    the physical fingertip link (``SHARPA_TACTILE_SENSOR_LINK_RIGHT``) — NOT the
    Ditto-matched ``*_retargeting_pad`` frame. We rotate it as::

        F_base = sign * R_base_link(q) @ R_link_sensor @ F6_xyz

    where ``R_base_link`` is FK of the mounting link and ``R_link_sensor`` is the
    (calibratable) residual sensor→link rotation.
    """

    name = "tactile"

    def __init__(
        self,
        sharpa_ik: SharpaFingerIK,
        fingers: tuple[FingerName, ...],
        *,
        force_sign: float = SHARPA_TACTILE_FORCE_SIGN,
        sensor_links: dict[str, str] | None = None,
        sensor_to_link_rpy: dict[str, tuple[float, float, float]] | None = None,
        debug: bool = False,
    ) -> None:
        self.sharpa_ik = sharpa_ik
        self.fingers = tuple(fingers)
        self.force_sign = float(force_sign)
        self.debug = bool(debug)
        self._sensor_link = sensor_links or SHARPA_TACTILE_SENSOR_LINK_RIGHT
        rpy = sensor_to_link_rpy or SHARPA_TACTILE_SENSOR_TO_LINK_RPY
        self._r_link_sensor = {
            finger: pin.rpy.rpyToMatrix(*rpy.get(finger, (0.0, 0.0, 0.0)))
            for finger in self.fingers
        }
        # Last good F6 per finger. Lets the loop poll faster than the sensor
        # streams (or with a near-zero timeout) without dropping to zero force on
        # a missed/None frame — we just reuse the most recent reading.
        self._last_f6: dict[str, np.ndarray] = {}

    def read(
        self, follower: "SharpaFollowerSession", q_seed_full: np.ndarray
    ) -> ForceRead | None:
        # Measured joint angles (for the link FK / downstream leader Jᵀ).
        inputs = follower.read_wrench_inputs(q_seed_full)
        if inputs is None:
            return None
        q_meas, _ = inputs

        forces: dict[str, np.ndarray] = {}
        for finger in self.fingers:
            f6 = follower.read_tactile_f6(finger)
            if f6 is None:
                # No fresh frame this cycle: reuse the last good reading instead
                # of dropping force to zero (avoids haptic chatter when polling
                # faster than the sensor streams).
                f6 = self._last_f6.get(finger)
                if f6 is None:
                    continue
            else:
                self._last_f6[finger] = np.asarray(f6, dtype=float)
            f_sensor = self.force_sign * np.asarray(f6[:3], dtype=float)
            link = self._sensor_link[finger]
            r_base_link = self.sharpa_ik.link_pose_in_base(q_meas, link).rotation
            f_base = r_base_link @ self._r_link_sensor[finger] @ f_sensor
            forces[finger] = f_base
            if self.debug:
                print(
                    f"[tactile:{finger}] F6_xyz=({f6[0]:+.2f},{f6[1]:+.2f},{f6[2]:+.2f}) "
                    f"-> base=({f_base[0]:+.2f},{f_base[1]:+.2f},{f_base[2]:+.2f})"
                )
        return q_meas, forces


class MixedForceSource:
    """Weighted blend of multiple sources (e.g. estimate ⊕ tactile)."""

    name = "mix"

    def __init__(self, sources: list[tuple[PadForceSource, float]]) -> None:
        if not sources:
            raise ValueError("MixedForceSource needs at least one source")
        self.sources = sources

    def read(
        self, follower: "SharpaFollowerSession", q_seed_full: np.ndarray
    ) -> ForceRead | None:
        q_meas: np.ndarray | None = None
        blended: dict[str, np.ndarray] = {}
        for source, weight in self.sources:
            result = source.read(follower, q_seed_full)
            if result is None:
                continue
            q_src, forces = result
            if q_meas is None:
                q_meas = q_src
            for finger, force in forces.items():
                if finger in blended:
                    blended[finger] = blended[finger] + weight * force
                else:
                    blended[finger] = weight * force
        if q_meas is None:
            return None
        return q_meas, blended


class CompositeForceSource:
    """Union of per-finger sources: each sub-source fills only its own fingers.

    Unlike ``MixedForceSource`` (which *sums* sources over the same fingers), this
    merges disjoint finger sets so different fingers can use different sources
    (e.g. index=estimate, thumb=tactile).
    """

    name = "composite"

    def __init__(self, sources: list[PadForceSource]) -> None:
        if not sources:
            raise ValueError("CompositeForceSource needs at least one source")
        self.sources = sources

    def read(
        self, follower: "SharpaFollowerSession", q_seed_full: np.ndarray
    ) -> ForceRead | None:
        q_meas: np.ndarray | None = None
        merged: dict[str, np.ndarray] = {}
        for source in self.sources:
            result = source.read(follower, q_seed_full)
            if result is None:
                continue
            q_src, forces = result
            if q_meas is None:
                q_meas = q_src
            merged.update(forces)
        if q_meas is None:
            return None
        return q_meas, merged


def make_force_source(
    sharpa_ik: SharpaFingerIK,
    fingers: tuple[FingerName, ...],
    mode: str,
    *,
    tactile_debug: bool = False,
) -> PadForceSource:
    """Build a pad-force source by name: ``estimate`` | ``tactile`` | ``mix``."""
    if mode == "estimate":
        return TorqueEstimateForceSource(sharpa_ik, fingers)
    if mode == "tactile":
        return TactileForceSource(sharpa_ik, fingers, debug=tactile_debug)
    if mode == "mix":
        return MixedForceSource(
            [
                (TorqueEstimateForceSource(sharpa_ik, fingers), 0.5),
                (TactileForceSource(sharpa_ik, fingers, debug=tactile_debug), 0.5),
            ]
        )
    raise ValueError(f"Unknown force mode {mode!r} (expected estimate|tactile|mix)")


def make_force_source_per_finger(
    sharpa_ik: SharpaFingerIK,
    finger_sources: dict[str, str],
    *,
    tactile_debug: bool = False,
) -> PadForceSource:
    """Build one pad-force source from a ``{finger: mode}`` map.

    Fingers are grouped by mode so each mode is built once over its finger
    subset; a single combined source is returned (a ``CompositeForceSource`` when
    more than one mode is in play). ``mode`` is ``estimate`` | ``tactile`` |
    ``mix``. An empty map yields an (unused) empty estimate source.
    """
    if not finger_sources:
        return TorqueEstimateForceSource(sharpa_ik, ())
    by_mode: dict[str, list[str]] = {}
    for finger, mode in finger_sources.items():
        by_mode.setdefault(mode, []).append(finger)
    sources = [
        make_force_source(sharpa_ik, tuple(fingers), mode, tactile_debug=tactile_debug)
        for mode, fingers in by_mode.items()
    ]
    return sources[0] if len(sources) == 1 else CompositeForceSource(sources)
