"""Sharpa Wave follower session — command retargeted ``sharpa_q`` to the hand.

Importing this module pulls in the Sharpa Wave SDK (via ``sharpa_hand``), so only
import it when Sharpa follower hardware is actually enabled.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from omegaconf import DictConfig

from _paths import setup_import_paths

setup_import_paths()

from sharpa_hand import (  # noqa: E402
    ANGLE_RANGES_DEG,
    JOINT_NAME_TO_INDEX,
    SharpaHand,
)

from hardware_interfaces.sharpa_follower.conventions import (  # noqa: E402
    SHARPA_FOLLOWER_SDK_JOINTS,
    SHARPA_TACTILE_CHANNEL_RIGHT,
    SHARPA_TORQUE_FEEDBACK_SIGN,
    SHARPA_URDF_TO_SDK_JOINT,
    SHARPA_URDF_TO_SDK_OFFSET_RAD,
    SHARPA_URDF_TO_SDK_SIGN,
    urdf_q_to_sdk_targets,
)
from retargeting.paths import (  # noqa: E402
    SHARPA_INDEX_JOINT_NAMES,
    SHARPA_THUMB_JOINT_NAMES,
)

_FINGER_URDF_JOINTS: dict[str, tuple[str, ...]] = {
    "index": SHARPA_INDEX_JOINT_NAMES,
    "thumb": SHARPA_THUMB_JOINT_NAMES,
}


class SharpaFollowerSession:
    """Drive the physical Sharpa hand from retargeted Sharpa URDF joint angles."""

    def __init__(self, config: DictConfig, *, verbose: bool = False) -> None:
        self.config = config
        self.verbose = verbose
        self.sharpa_hand: SharpaHand | None = None
        self._q_index: dict[str, int] | None = None
        self._finger_sdk_idx: dict[str, list[int]] = {}
        self._started = False
        self._tactile_ready = False
        self.tactile_timeout_s = float(
            config.sharpa.get("tactile_timeout_s", 0.005)
        )

    @property
    def is_connected(self) -> bool:
        return self._started

    def start(self, q_index_of: Callable[[str], int]) -> None:
        """Connect, enable index/thumb joints, and start position streaming.

        ``q_index_of`` maps a URDF joint name to its index in the retargeted
        ``sharpa_q`` (e.g. ``SharpaFingerIK.joint_q_index``).
        """
        self._apply_angle_overrides()
        hand = SharpaHand.from_config(self.config.sharpa, verbose=self.verbose)
        hand.set_enabled_joints(list(SHARPA_FOLLOWER_SDK_JOINTS))
        hand.connect()
        hand.configure()
        hand.start()
        self.sharpa_hand = hand
        self._q_index = {name: q_index_of(name) for name in SHARPA_URDF_TO_SDK_JOINT}
        self._finger_sdk_idx = {
            finger: [
                JOINT_NAME_TO_INDEX[SHARPA_URDF_TO_SDK_JOINT[name]] for name in joints
            ]
            for finger, joints in _FINGER_URDF_JOINTS.items()
        }
        self._started = True
        if self.verbose:
            print(f"  Sharpa follower joints: {list(SHARPA_FOLLOWER_SDK_JOINTS)}")

    def _apply_angle_overrides(self) -> None:
        """Widen the SDK position-clamp limits in place from config (deg).

        Mutates the shared ``ANGLE_RANGES_DEG`` table that the SDK consults when
        clamping ``send_positions`` targets, so we can extend ROM (e.g. thumb)
        without editing the sharpa_controller submodule.
        """
        overrides = self.config.sharpa.get("angle_overrides_deg")
        if not overrides:
            return
        for name, rng in overrides.items():
            if name not in JOINT_NAME_TO_INDEX:
                raise ValueError(
                    f"Unknown Sharpa joint in angle_overrides_deg: {name!r}"
                )
            lo, hi = float(rng[0]), float(rng[1])
            ANGLE_RANGES_DEG[JOINT_NAME_TO_INDEX[name]] = (lo, hi)
            if self.verbose:
                print(f"  Sharpa ROM override {name}: [{lo:.0f}, {hi:.0f}] deg")

    def send_q(self, sharpa_q: np.ndarray) -> None:
        """Send a retargeted Sharpa URDF configuration to the hand (no-op if idle)."""
        if not self._started or self.sharpa_hand is None or self._q_index is None:
            return
        targets = urdf_q_to_sdk_targets(sharpa_q, self._q_index.__getitem__)
        self.sharpa_hand.send_positions(targets)

    def read_wrench_inputs(
        self,
        q_seed_full: np.ndarray,
    ) -> tuple[np.ndarray, dict[str, np.ndarray]] | None:
        """Read measured state for wrench estimation (no-op if idle).

        Returns ``(q_full, finger_torques)`` where ``q_full`` is ``q_seed_full``
        with the driven index/thumb joints overwritten by measured angles (URDF
        convention), and ``finger_torques`` maps finger → joint torques (Nm) in
        ``SHARPA_*_JOINT_NAMES`` order, sign-corrected to the URDF convention.
        """
        if not self._started or self.sharpa_hand is None or self._q_index is None:
            return None
        state = self.sharpa_hand.read_state()
        angles = np.asarray(state.angles, dtype=float)
        torques = np.asarray(state.torques, dtype=float)
    
        q = np.asarray(q_seed_full, dtype=float).copy()
        finger_torques: dict[str, np.ndarray] = {}
        for finger, joints in _FINGER_URDF_JOINTS.items():
            taus = []
            for name, sdk_i in zip(joints, self._finger_sdk_idx[finger]):
                sign = SHARPA_URDF_TO_SDK_SIGN[name]
                offset = SHARPA_URDF_TO_SDK_OFFSET_RAD[name]
                q[self._q_index[name]] = (angles[sdk_i] - offset) / sign
                taus.append(SHARPA_TORQUE_FEEDBACK_SIGN * sign * torques[sdk_i])
            finger_torques[finger] = np.asarray(taus, dtype=float)
        return q, finger_torques

    def enable_tactile(self, *, calibrate: bool = False) -> bool:
        """Verify the hand exposes fingertip tactile sensors (optionally calibrate).

        Returns ``True`` if tactile is available. Safe to call after ``start()``
        (the SDK device stream is already running).
        """
        if not self._started or self.sharpa_hand is None:
            return False
        device = self.sharpa_hand.hand
        info_fn = getattr(self.sharpa_hand.hand, "get_device_info", None)
        if info_fn is not None:
            info = info_fn()
            checker = getattr(info, "has_fingertip_tactile", None)
            has_tactile = bool(checker() if callable(checker) else checker)
            if not has_tactile:
                if self.verbose:
                    print("  Sharpa device reports no fingertip tactile sensors.")
                return False
        if calibrate:
            if self.verbose:
                print("  Calibrating Sharpa tactile sensors...")
            calib = getattr(device, "calib_tactile", None)
            if calib is not None and not calib():
                print("  Tactile calibration failed.")
                return False
        # Ask the underlying SharpaHand to cache tactile frames on its IO loop.
        enabled = False
        try:
            enabled = self.sharpa_hand.enable_tactile(SHARPA_TACTILE_CHANNEL_RIGHT, timeout_s=self.tactile_timeout_s)
        except Exception:
            enabled = False
        self._tactile_ready = bool(enabled)
        if self.verbose and self._tactile_ready:
            print(f"  Sharpa tactile enabled (cached): {SHARPA_TACTILE_CHANNEL_RIGHT}")
        return self._tactile_ready

    def read_tactile_f6(self, finger: str) -> np.ndarray | None:
        """Latest 6-axis tactile wrench ``[Fx,Fy,Fz,Tx,Ty,Tz]`` (sensor frame).

        Returns ``None`` if tactile is unavailable or no frame is ready. The
        force triplet is in the fingertip sensor frame (not yet base-aligned).
        """
        if not self._tactile_ready or self.sharpa_hand is None:
            return None
        channel = SHARPA_TACTILE_CHANNEL_RIGHT.get(finger)
        if channel is None:
            return None
        # Read the latest cached frame from the SharpaHand IO loop (non-blocking).
        cached = self.sharpa_hand.get_latest_tactile(finger)
        if not cached or "content" not in cached:
            return None
        f6 = cached["content"].get("F6")
        if f6 is None:
            return None
        return np.asarray(f6, dtype=float)

    def stop(self) -> None:
        """Stop streaming and disconnect (idempotent)."""
        if self.sharpa_hand is not None:
            self.sharpa_hand.disconnect()
            self.sharpa_hand = None
        self._started = False
        self._tactile_ready = False
