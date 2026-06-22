"""Leader Dynamixel + Sharpa Wave follower teleoperation."""

from __future__ import annotations

import queue
import time
from dataclasses import dataclass
from typing import Optional

from ._paths import setup_import_paths

setup_import_paths()

from hand_interfaces.src.current_control import (  # noqa: E402
    ControlParams,
    CurrentController,
    create_control_params_from_config,
)
from hand_interfaces.src.hand_interface import create_hand_interface  # noqa: E402
from hand_interfaces.src.motor_data import MotorData  # noqa: E402
from sharpa_hand import JOINT_NAME_TO_INDEX, SharpaHand  # noqa: E402
from utils.utils import precise_sleep  # noqa: E402


@dataclass
class JointPair:
    """Maps one leader motor to one Sharpa joint."""

    leader_motor_id: int
    sharpa_joint: str
    offset_rad: float = 0.0
    scale: float = 1.0
    torque_to_mA: float = 50.0
    torque_filter_alpha: float = 1.0

    @property
    def sharpa_index(self) -> int:
        return JOINT_NAME_TO_INDEX[self.sharpa_joint]

    @property
    def filtering_enabled(self) -> bool:
        return self.torque_filter_alpha < 1.0


class SharpaDittoTeleop:
    """Dynamixel leader (current) teleoperates Sharpa hand (position) with torque feedback."""

    def __init__(
        self,
        config,
        verbose: bool | None = None,
        state_queue: Optional[queue.Queue] = None,
    ):
        self.config = config
        self.state_queue = state_queue
        self.verbose = (
            verbose
            if verbose is not None
            else bool(config.get("verbose", config.hand_config.get("verbose", True)))
        )
        self.control_frequency = float(
            config.hand_config.get(
                "control_frequency", config.get("control_frequency", 100)
            )
        )
        self.dt = 1.0 / self.control_frequency
        self.show_current_breakdown = bool(
            config.hand_config.get(
                "show_current_breakdown",
                config.get("show_current_breakdown", False),
            )
        )

        leader_cfg = config.hand_config.leader
        if leader_cfg.mode != "current":
            raise ValueError(
                f"SharpaDittoTeleop requires leader mode 'current', got {leader_cfg.mode!r}"
            )

        self.leader_chain = list(leader_cfg.motor_ids)
        self.pairs = self._parse_joint_pairs()
        self._validate_pairs()

        self.leader_hand = None
        self.sharpa_hand: SharpaHand | None = None
        self.current_controller = CurrentController()
        self.leader_control_params = self._create_leader_control_params()
        self._filtered_torque_nm: dict[int, float] = {}
        self.running = False

    def _parse_joint_pairs(self) -> list[JointPair]:
        mapping = self.config.hand_config.get("sharpa_mapping")
        if mapping is None or not mapping.get("pairs"):
            raise ValueError("hand_config.sharpa_mapping.pairs is required")

        pairs = []
        for entry in mapping.pairs:
            pairs.append(
                JointPair(
                    leader_motor_id=int(entry.leader_motor_id),
                    sharpa_joint=str(entry.sharpa_joint),
                    offset_rad=float(entry.get("offset_rad", 0.0)),
                    scale=float(entry.get("scale", 1.0)),
                    torque_to_mA=float(entry.get("torque_to_mA", 50.0)),
                    torque_filter_alpha=float(entry.get("torque_filter_alpha", 1.0)),
                )
            )
        return pairs

    def _validate_pairs(self) -> None:
        if len(self.pairs) != len(self.leader_chain):
            raise ValueError(
                f"Expected {len(self.leader_chain)} sharpa_mapping pairs, "
                f"got {len(self.pairs)}"
            )

        leader_ids = set(self.leader_chain)
        for pair in self.pairs:
            if pair.leader_motor_id not in leader_ids:
                raise ValueError(
                    f"sharpa_mapping references unknown leader motor "
                    f"{pair.leader_motor_id}"
                )

    def _create_leader_control_params(self) -> list[ControlParams]:
        control_params = []
        leader_config = self.config.hand_config.leader
        disable_global = leader_config.get("disable_force_rendering_global", False)

        for motor_id in self.leader_chain:
            motor_config = {}
            if leader_config.get("joint_settings"):
                for joint_config in leader_config.joint_settings:
                    if (
                        joint_config["motor_id"] == motor_id
                        and "current_control" in joint_config
                    ):
                        motor_config = joint_config["current_control"]
                        break

            if motor_config:
                params = create_control_params_from_config(motor_id, motor_config)
            else:
                params = ControlParams(motor_id=motor_id)

            if disable_global:
                params.enable_force_rendering = False

            control_params.append(params)

            if self.verbose:
                ffd = ""
                if params.enable_force_rendering_damping:
                    ffd = (
                        f", force_rendering_damping gain="
                        f"{params.force_rendering_damping_gain}, "
                        f"max={params.force_rendering_damping_max_current:.1f} mA"
                    )
                print(
                    f"  Leader motor {motor_id}: force_rendering="
                    f"{params.enable_force_rendering}, gain={params.force_rendering_gain}"
                    f"{ffd}"
                )

        return control_params

    def _get_u2d2_port_baud(self) -> tuple[str, int]:
        u2d2_config = self.config.u2d2
        port = u2d2_config.get("usb_port", "/dev/ttyUSB0")
        baudrate = int(u2d2_config.get("baudrate", 4000000))
        return port, baudrate

    def connect(self) -> None:
        port, baudrate = self._get_u2d2_port_baud()
        use_fake = bool(self.config.u2d2.get("fake_u2d2", False))

        if self.verbose:
            print(f"Connecting Dynamixel leader on {port} @ {baudrate}...")

        self.leader_hand = create_hand_interface(
            self.config,
            "leader",
            "real",
            port=port,
            baudrate=baudrate,
            use_fake=use_fake,
        )
        self.leader_hand.connect()

        self.sharpa_hand = SharpaHand.from_config(
            self.config.sharpa, verbose=self.verbose
        )
        # Prefer joints from hand_config mapping so each hand_config is self-contained.
        mapped_joints = [pair.sharpa_joint for pair in self.pairs]
        enabled = mapped_joints or list(self.config.sharpa.get("enabled_joints", []))
        if enabled:
            self.sharpa_hand.set_enabled_joints(enabled)
            if self.verbose:
                print(f"  Sharpa enabled joints: {enabled}")
        self.sharpa_hand.connect()
        self.sharpa_hand.configure()
        self.sharpa_hand.start()
        self._filtered_torque_nm.clear()

        if self.verbose:
            print("Connected leader Dynamixel and Sharpa hand.")
            for pair in self.pairs:
                if pair.filtering_enabled:
                    print(
                        f"  Torque input filter motor {pair.leader_motor_id}: "
                        f"alpha={pair.torque_filter_alpha}"
                    )

    def setup_motors(self) -> None:
        if self.leader_hand is None:
            raise RuntimeError("Leader not connected. Call connect() first.")
        self.leader_hand.setup_motors()

    def disconnect(self) -> None:
        if self.leader_hand is not None:
            self.leader_hand.disconnect()
            self.leader_hand = None
        if self.sharpa_hand is not None:
            self.sharpa_hand.disconnect()
            self.sharpa_hand = None

    def _leader_index_for_motor(self, motor_id: int) -> int:
        return self.leader_chain.index(motor_id)

    def _leader_to_sharpa_angle(self, pair: JointPair, leader_joint_angle: float) -> float:
        return pair.offset_rad + pair.scale * leader_joint_angle

    def _filter_torque_nm(self, pair: JointPair, torque_nm: float) -> float:
        """EMA input filter on Sharpa torque before force-rendering mapping."""
        alpha = pair.torque_filter_alpha
        if alpha >= 1.0:
            return torque_nm

        motor_id = pair.leader_motor_id
        prev = self._filtered_torque_nm.get(motor_id)
        if prev is None:
            filtered = torque_nm
        else:
            filtered = alpha * torque_nm + (1.0 - alpha) * prev
        self._filtered_torque_nm[motor_id] = filtered
        return filtered

    def _build_synthetic_follower_data(
        self,
        sharpa_state,
        leader_motor_data: list[MotorData],
    ) -> tuple[list[MotorData], list[float], list[float], list[float], list[float]]:
        """Map Sharpa torques to pseudo follower currents for force rendering.

        Returns:
            follower MotorData (uses filtered synth mA),
            raw torques (Nm), filtered torques (Nm),
            raw synth mA, filtered synth mA
        """
        follower_data: list[MotorData] = []
        raw_torques: list[float] = []
        filtered_torques: list[float] = []
        raw_currents: list[float] = []
        filtered_currents: list[float] = []

        for pair in self.pairs:
            idx = pair.sharpa_index
            torque_raw = sharpa_state.torques[idx]
            torque_filt = self._filter_torque_nm(pair, torque_raw)
            synth_raw = torque_raw * pair.torque_to_mA
            synth_filt = torque_filt * pair.torque_to_mA

            raw_torques.append(torque_raw)
            filtered_torques.append(torque_filt)
            raw_currents.append(synth_raw)
            filtered_currents.append(synth_filt)

            follower_data.append(
                MotorData(
                    motor_id=pair.leader_motor_id,
                    raw_position=0,
                    raw_velocity=0,
                    raw_current=0,
                    position=sharpa_state.angles[idx],
                    velocity=sharpa_state.velocities[idx],
                    current=synth_filt,
                    joint_angle=sharpa_state.angles[idx],
                    joint_velocity=sharpa_state.velocities[idx],
                )
            )
        return (
            follower_data,
            raw_torques,
            filtered_torques,
            raw_currents,
            filtered_currents,
        )

    def _queue_states(
        self,
        leader_motor_data: list[MotorData],
        sharpa_state,
        follower_motor_data: list[MotorData],
        current_commands: list[float],
        raw_torques: list[float],
        filtered_torques: list[float],
        raw_synth_currents: list[float],
        filtered_synth_currents: list[float],
        force_rendering_damping_currents: list[float] | None = None,
    ) -> None:
        if self.state_queue is None:
            return

        states_data = {
            "leader": {
                "currents": [d.current for d in leader_motor_data],
                "joint_velocities": [d.joint_velocity for d in leader_motor_data],
                "joint_angles": [d.joint_angle for d in leader_motor_data],
                "command_currents": list(current_commands),
                "force_rendering_damping_currents": list(
                    force_rendering_damping_currents or []
                ),
            },
            "follower": {
                "currents": filtered_synth_currents,
                "currents_raw": raw_synth_currents,
                "joint_velocities": [d.joint_velocity for d in follower_motor_data],
                "joint_angles": [d.joint_angle for d in follower_motor_data],
                "torques_nm": raw_torques,
                "torques_nm_filtered": filtered_torques,
            },
        }
        try:
            self.state_queue.put_nowait(states_data)
        except queue.Full:
            if self.verbose:
                print("State queue full, dropping message")

    def _control_step(self) -> None:
        leader_motor_data = self.leader_hand.read_states()
        sharpa_state = self.sharpa_hand.read_state()

        targets = {}
        for pair in self.pairs:
            leader_idx = self._leader_index_for_motor(pair.leader_motor_id)
            leader_angle = leader_motor_data[leader_idx].joint_angle
            targets[pair.sharpa_joint] = self._leader_to_sharpa_angle(
                pair, leader_angle
            )
        self.sharpa_hand.send_positions(targets)

        (
            follower_motor_data,
            raw_torques,
            filtered_torques,
            raw_synth_currents,
            filtered_synth_currents,
        ) = self._build_synthetic_follower_data(sharpa_state, leader_motor_data)
        current_commands = self.current_controller.compute_bulk_current_commands(
            leader_motor_data,
            self.leader_control_params,
            time.time(),
            follower_motor_data,
            self.show_current_breakdown,
        )
        force_rendering_damping_cmds = (
            self.current_controller.last_force_rendering_damping_cmds
        )
        self._queue_states(
            leader_motor_data,
            sharpa_state,
            follower_motor_data,
            current_commands,
            raw_torques,
            filtered_torques,
            raw_synth_currents,
            filtered_synth_currents,
            force_rendering_damping_cmds,
        )
        self.leader_hand.send_current_commands(
            [int(round(c)) for c in current_commands]
        )

    def start_control_loop(self) -> None:
        if self.leader_hand is None or self.sharpa_hand is None:
            raise RuntimeError("Not connected. Call connect() first.")

        print(f"\nStarting SharpaDitto teleop at {self.control_frequency} Hz")
        print(f"  Leader motors: {self.leader_chain} (current mode)")
        print(f"  Sharpa joints: {[p.sharpa_joint for p in self.pairs]}")
        if self.sharpa_hand is not None:
            print(f"  Sharpa IO: {self.sharpa_hand.io_frequency_hz} Hz (independent thread)")

        self.running = True
        try:
            while self.running:
                cycle_start = time.perf_counter()
                self._control_step()
                elapsed = time.perf_counter() - cycle_start
                precise_sleep(max(0.0, self.dt - elapsed))
        except KeyboardInterrupt:
            print("\nStopping teleop...")
        finally:
            self.running = False

    def stop(self) -> None:
        self.running = False
