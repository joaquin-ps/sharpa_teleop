"""Ditto leader hardware session for retargeting teleop."""

from __future__ import annotations

import queue
import threading
import time

import numpy as np
from omegaconf import DictConfig

from _paths import setup_import_paths

setup_import_paths()

from ditto.controllers.teleop_controller import TeleopHand
from hardware_interfaces.ditto_leader.conventions import (
    hardware_joint_angles_to_urdf,
    leader_joint_names_for_motor_ids,
)
from retargeting.paths import (
    DITTO_3F_LEADER_MOTOR_IDS,
    DITTO_LEADER_MOTOR_IDS,
    DITTO_LEADER_ONLY_HAND_CONFIG,
    DITTO_3F_LEADER_ONLY_HAND_CONFIG,
)


class LeaderHardwareSession:
    """Leader-only TeleopHand session that publishes URDF-ordered joint angles."""

    def __init__(self, config: DictConfig) -> None:
        if TeleopHand.is_idle_hand_config(config):
            raise ValueError(
                "Leader hardware requires a physical hand_config "
                f"(e.g. hand_config={DITTO_LEADER_ONLY_HAND_CONFIG} or "
                f"{DITTO_3F_LEADER_ONLY_HAND_CONFIG})."
            )
        self.config = config
        self.leader_joint_names = leader_joint_names_for_motor_ids(
            config.hand_config.leader.motor_ids
        )
        self.state_queue: queue.Queue = queue.Queue(maxsize=20)
        self.teleop_hand: TeleopHand | None = None
        self._control_thread: threading.Thread | None = None
        self._last_joint_angles: np.ndarray | None = None
        self._read_failure_count = 0
        self._warned_read_failure = False

    @property
    def is_receiving(self) -> bool:
        """True after at least one valid leader sample has been received."""
        return self._last_joint_angles is not None

    def _warn_read_failure(self, detail: str) -> None:
        self._read_failure_count += 1
        if self._warned_read_failure:
            return
        self._warned_read_failure = True
        port = self.config.u2d2.get("usb_port", "/dev/ttyUSB0")
        motor_ids = list(self.config.hand_config.leader.motor_ids)
        print(
            "\n⚠️  Ditto leader hardware read failed "
            f"({detail}).\n"
            f"   Check USB ({port}), power, and motor IDs {motor_ids}.\n"
            "   Viewer will keep running; use sliders until reads succeed.\n"
            "   Override port: python sharpa_teleop/viz/view_teleop.py u2d2.usb_port=/dev/ttyUSB1\n"
        )

    @staticmethod
    def validate_config(config: DictConfig) -> None:
        if not (
            config.hand_config.leader.mode == "torque_off"
            and config.hand_config.follower.mode == "none"
        ):
            raise ValueError(
                "Leader-only retargeting expects follower.mode=none and "
                "leader.mode=torque_off (no force feedback)."
            )
        motor_ids = list(config.hand_config.leader.motor_ids)
        if motor_ids not in (
            list(DITTO_LEADER_MOTOR_IDS),
            list(DITTO_3F_LEADER_MOTOR_IDS),
        ):
            leader_joint_names_for_motor_ids(motor_ids)

    def start(self) -> None:
        """Connect, enable motors, and start the background read loop."""
        self.validate_config(self.config)
        self.teleop_hand = TeleopHand(self.config, state_queue=self.state_queue)
        if not self.teleop_hand.is_leader_only:
            raise RuntimeError("Expected leader-only TeleopHand configuration.")

        self.teleop_hand.connect()
        self.teleop_hand.setup_motors()
        self._control_thread = threading.Thread(
            target=self.teleop_hand.start_control_loop,
            daemon=True,
            name="ditto-leader-control",
        )
        self._control_thread.start()
        time.sleep(0.2)

    def poll_joint_angles(self) -> np.ndarray | None:
        """Return the latest leader joint angles (rad) in URDF joint order."""
        if self.teleop_hand is None:
            return self._last_joint_angles

        states_data = None
        while True:
            try:
                states_data = self.state_queue.get_nowait()
            except queue.Empty:
                break

        if states_data is None:
            return self._last_joint_angles

        leader_states = states_data.get("leader") or {}
        joint_angles = leader_states.get("joint_angles")
        if joint_angles is None or len(joint_angles) == 0:
            return self._last_joint_angles

        angles = np.asarray(joint_angles, dtype=float)
        expected = len(self.leader_joint_names)
        if angles.shape[0] != expected:
            self._warn_read_failure(
                f"expected {expected} joint angles, got {angles.shape[0]}"
            )
            return self._last_joint_angles

        self._read_failure_count = 0
        self._last_joint_angles = hardware_joint_angles_to_urdf(
            angles, self.leader_joint_names
        )
        return self._last_joint_angles

    def stop(self) -> None:
        """Stop the control loop and disconnect."""
        if self.teleop_hand is not None:
            self.teleop_hand.running = False
        if self._control_thread is not None:
            self._control_thread.join(timeout=2.0)
            self._control_thread = None
        if self.teleop_hand is not None:
            self.teleop_hand.disconnect()
            self.teleop_hand = None
