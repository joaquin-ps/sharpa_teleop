#!/usr/bin/env python3
"""
Live plot tool for Sharpa + Dynamixel teleop.

Plots leader (Dynamixel) vs follower (Sharpa) states in real time to diagnose
oscillations, mapping error, and force-feedback behavior.

Usage (from repo root):
    python sharpa_teleop/live_plot.py
    python sharpa_teleop/live_plot.py u2d2.usb_port=/dev/ttyUSB0

From sharpa_teleop/ package dir:
    python live_plot.py

Optional flags (stripped before Hydra):
    python sharpa_teleop/live_plot.py --position
    python sharpa_teleop/live_plot.py --current --velocity --torque
"""

from __future__ import annotations

import queue
import sys
import threading
import time
from collections import deque
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import DictConfig

from sharpa_teleop._paths import CONF_DIR, FA_CONF_DIR  # noqa: E402
from sharpa_teleop.sharpa_ditto_teleop import SharpaDittoTeleop  # noqa: E402


def _build_overrides(cli_args: list[str]) -> list[str]:
    searchpath = f"hydra.searchpath=[file://{FA_CONF_DIR}]"
    return [searchpath] + list(cli_args)


def _parse_plot_flags() -> list[str] | None:
    plot_flags = {
        "--current": "current",
        "--velocity": "velocity",
        "--position": "position",
        "--torque": "torque",
    }
    selected = []
    remaining = []
    for arg in sys.argv[1:]:
        if arg in plot_flags:
            selected.append(plot_flags[arg])
        else:
            remaining.append(arg)
    sys.argv[1:] = remaining
    return selected if selected else None


class SharpaLivePlotter:
    """Live plotter for SharpaDittoTeleop."""

    PLOT_TYPES = ["current", "velocity", "position", "torque"]

    def __init__(
        self,
        config: DictConfig,
        max_history: int = 2000,
        plot_window_seconds: float = 10.0,
        plot_types: list[str] | None = None,
    ):
        self.config = config
        self.max_history = max_history
        self.plot_window_seconds = plot_window_seconds

        if plot_types:
            self.plot_types = [p for p in plot_types if p in self.PLOT_TYPES]
        else:
            self.plot_types = ["current", "velocity", "position"]
        self.num_plot_types = len(self.plot_types)

        self.leader_chain = list(config.hand_config.leader.motor_ids)
        mapping = config.hand_config.sharpa_mapping.pairs
        self.follower_labels = [str(p.sharpa_joint) for p in mapping]
        self.num_joints = len(self.leader_chain)

        leader_config = config.hand_config.leader
        self.thresholds_positive = []
        self.thresholds_negative = []
        threshold_pos_map = {}
        threshold_neg_map = {}
        if leader_config.get("joint_settings"):
            for joint_config in leader_config.joint_settings:
                motor_id = joint_config.get("motor_id")
                if motor_id is not None and "current_control" in joint_config:
                    cc = joint_config["current_control"]
                    base = cc.get("force_rendering_threshold", 50.0)
                    threshold_pos_map[motor_id] = cc.get(
                        "force_rendering_threshold_positive", base
                    )
                    threshold_neg_map[motor_id] = cc.get(
                        "force_rendering_threshold_negative", base
                    )
        for leader_motor_id in self.leader_chain:
            self.thresholds_positive.append(
                threshold_pos_map.get(leader_motor_id, 50.0)
            )
            self.thresholds_negative.append(
                threshold_neg_map.get(leader_motor_id, 50.0)
            )

        self.time_data: deque[float] = deque(maxlen=max_history)
        self.leader_currents = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.leader_command_currents = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.leader_force_rendering_damping_currents = [
            deque(maxlen=max_history) for _ in range(self.num_joints)
        ]
        self.follower_currents = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.follower_currents_raw = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.leader_velocities = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.follower_velocities = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.leader_positions = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.follower_positions = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.follower_torques = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.follower_torques_filtered = [deque(maxlen=max_history) for _ in range(self.num_joints)]

        self.state_queue: queue.Queue = queue.Queue(maxsize=20)
        self.running = False
        self.teleop: SharpaDittoTeleop | None = None
        self.scale_counter = 0
        self.scale_interval = 10

        self.setup_plots()

    def setup_plots(self) -> None:
        num_cols = self.num_plot_types
        self.fig, axes = plt.subplots(
            self.num_joints,
            num_cols,
            figsize=(5 * num_cols, 4 * self.num_joints),
            squeeze=False,
        )
        self.col_map = {ptype: col for col, ptype in enumerate(self.plot_types)}
        self.axes = axes

        for joint_idx in range(self.num_joints):
            leader_id = self.leader_chain[joint_idx]
            follower_label = (
                self.follower_labels[joint_idx]
                if joint_idx < len(self.follower_labels)
                else "Sharpa"
            )

            if "current" in self.col_map:
                col = self.col_map["current"]
                ax = self.axes[joint_idx, col]
                ax.set_title(
                    f"Joint {joint_idx} - Synth current (mA)\n"
                    f"raw vs filtered input to force rendering"
                )
                ax.set_ylabel("Current (mA)")
                ax.grid(True, alpha=0.3)
                threshold_pos = self.thresholds_positive[joint_idx]
                threshold_neg = self.thresholds_negative[joint_idx]
                ax.axhline(y=threshold_pos, color="r", linestyle="--", alpha=0.7)
                ax.axhline(y=-threshold_neg, color="r", linestyle="--", alpha=0.7)
                ax.set_ylim(-300, 300)

            if "velocity" in self.col_map:
                col = self.col_map["velocity"]
                ax = self.axes[joint_idx, col]
                ax.set_title(f"Joint {joint_idx} - Velocity (rad/s)")
                ax.set_ylabel("Velocity (rad/s)")
                ax.grid(True, alpha=0.3)
                ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
                ax.set_ylim(-2.0, 2.0)

            if "position" in self.col_map:
                col = self.col_map["position"]
                ax = self.axes[joint_idx, col]
                ax.set_title(
                    f"Joint {joint_idx} - Position (rad)\n"
                    f"Leader vs {follower_label}"
                )
                if joint_idx == self.num_joints - 1:
                    ax.set_xlabel("Time (s)")
                ax.set_ylabel("Position (rad)")
                ax.grid(True, alpha=0.3)
                ax.set_ylim(-0.5, 1.0)

            if "torque" in self.col_map:
                col = self.col_map["torque"]
                ax = self.axes[joint_idx, col]
                ax.set_title(
                    f"Joint {joint_idx} - Sharpa torque (Nm)\n"
                    f"raw vs filtered ({follower_label})"
                )
                ax.set_ylabel("Torque (Nm)")
                ax.grid(True, alpha=0.3)
                ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
                ax.set_ylim(-0.5, 0.5)

        plt.tight_layout()

        self.leader_current_lines = []
        self.leader_command_lines = []
        self.leader_force_rendering_damping_lines = []
        self.follower_current_lines = []
        self.follower_current_lines_raw = []
        self.leader_velocity_lines = []
        self.follower_velocity_lines = []
        self.leader_position_lines = []
        self.follower_position_lines = []
        self.follower_torque_lines = []
        self.follower_torque_lines_filtered = []

        for joint_idx in range(self.num_joints):
            if "current" in self.col_map:
                col = self.col_map["current"]
                ax = self.axes[joint_idx, col]
                line_l, = ax.plot([], [], "b-", label="Leader meas", linewidth=1.5)
                line_c, = ax.plot([], [], "c--", label="Leader cmd", linewidth=1.2)
                line_fr, = ax.plot([], [], "g-", label="Synth raw", linewidth=1.2, alpha=0.7)
                line_ff, = ax.plot(
                    [], [], color="orange", linestyle="--",
                    label="Synth filt", linewidth=1.5,
                )
                line_fd, = ax.plot(
                    [], [], color="purple", linestyle=":",
                    label="Force rendering damping", linewidth=1.5,
                )
                self.leader_current_lines.append(line_l)
                self.leader_command_lines.append(line_c)
                self.follower_current_lines_raw.append(line_fr)
                self.follower_current_lines.append(line_ff)
                self.leader_force_rendering_damping_lines.append(line_fd)

            if "velocity" in self.col_map:
                col = self.col_map["velocity"]
                ax = self.axes[joint_idx, col]
                line_l, = ax.plot([], [], "b-", label="Leader", linewidth=1.5)
                line_f, = ax.plot([], [], "g-", label="Sharpa", linewidth=1.5)
                self.leader_velocity_lines.append(line_l)
                self.follower_velocity_lines.append(line_f)

            if "position" in self.col_map:
                col = self.col_map["position"]
                ax = self.axes[joint_idx, col]
                line_l, = ax.plot([], [], "b-", label="Leader", linewidth=1.5)
                line_f, = ax.plot([], [], "g-", label="Sharpa", linewidth=1.5)
                self.leader_position_lines.append(line_l)
                self.follower_position_lines.append(line_f)

            if "torque" in self.col_map:
                col = self.col_map["torque"]
                ax = self.axes[joint_idx, col]
                line_r, = ax.plot([], [], "m-", label="Torque raw", linewidth=1.2, alpha=0.7)
                line_f, = ax.plot(
                    [], [], color="orange", linestyle="--",
                    label="Torque filt", linewidth=1.5,
                )
                self.follower_torque_lines.append(line_r)
                self.follower_torque_lines_filtered.append(line_f)

        self.axes[0, 0].legend(loc="upper right", fontsize=8)

    def data_collection_thread(self) -> None:
        start_time = time.time()
        while self.running:
            try:
                states_data = None
                while True:
                    try:
                        states_data = self.state_queue.get_nowait()
                    except queue.Empty:
                        break
                if states_data is None:
                    states_data = self.state_queue.get(timeout=0.1)

                current_time = time.time() - start_time
                self.time_data.append(current_time)

                leader = states_data.get("leader", {})
                follower = states_data.get("follower", {})

                leader_currents = leader.get("currents", [])
                leader_commands = leader.get("command_currents", [])
                leader_force_rendering_damping = leader.get(
                    "force_rendering_damping_currents", []
                )
                follower_currents = follower.get("currents", [])
                follower_currents_raw = follower.get("currents_raw", follower_currents)
                leader_velocities = leader.get("joint_velocities", [])
                follower_velocities = follower.get("joint_velocities", [])
                leader_positions = leader.get("joint_angles", [])
                follower_positions = follower.get("joint_angles", [])
                follower_torques = follower.get("torques_nm", [])
                follower_torques_filtered = follower.get(
                    "torques_nm_filtered", follower_torques
                )

                for joint_idx in range(self.num_joints):
                    self._append_or_zero(
                        self.leader_currents[joint_idx], leader_currents, joint_idx
                    )
                    self._append_or_zero(
                        self.leader_command_currents[joint_idx], leader_commands, joint_idx
                    )
                    self._append_or_zero(
                        self.leader_force_rendering_damping_currents[joint_idx],
                        leader_force_rendering_damping,
                        joint_idx,
                    )
                    self._append_or_zero(
                        self.follower_currents[joint_idx], follower_currents, joint_idx
                    )
                    self._append_or_zero(
                        self.follower_currents_raw[joint_idx],
                        follower_currents_raw,
                        joint_idx,
                    )
                    self._append_or_zero(
                        self.leader_velocities[joint_idx], leader_velocities, joint_idx
                    )
                    self._append_or_zero(
                        self.follower_velocities[joint_idx], follower_velocities, joint_idx
                    )
                    self._append_or_zero(
                        self.leader_positions[joint_idx], leader_positions, joint_idx
                    )
                    self._append_or_zero(
                        self.follower_positions[joint_idx], follower_positions, joint_idx
                    )
                    self._append_or_zero(
                        self.follower_torques[joint_idx], follower_torques, joint_idx
                    )
                    self._append_or_zero(
                        self.follower_torques_filtered[joint_idx],
                        follower_torques_filtered,
                        joint_idx,
                    )
            except queue.Empty:
                continue
            except Exception as exc:
                print(f"Error in data collection thread: {exc}")
                time.sleep(0.01)

    @staticmethod
    def _append_or_zero(deq: deque, values: list, idx: int) -> None:
        if idx < len(values):
            deq.append(values[idx])
        else:
            deq.append(0.0)

    def update_plots(self, _frame):
        if not self.running or len(self.time_data) == 0:
            return []

        control_freq = self.config.hand_config.get(
            "control_frequency", self.config.get("control_frequency", 100)
        )
        plot_window_size = int(self.plot_window_seconds * control_freq)

        time_array = np.array(list(self.time_data))
        if len(time_array) > plot_window_size:
            time_array = time_array[-plot_window_size:]
            plot_length = plot_window_size
        else:
            plot_length = len(time_array)

        if plot_length == 0:
            return []

        all_lines = []
        should_scale = self.scale_counter % self.scale_interval == 0
        self.scale_counter += 1

        for joint_idx in range(self.num_joints):
            time_plot, arrays = self._trim_series(joint_idx, time_array, plot_length)

            if "current" in self.col_map:
                self.leader_current_lines[joint_idx].set_data(
                    time_plot, arrays["leader_current"]
                )
                self.leader_command_lines[joint_idx].set_data(
                    time_plot, arrays["leader_command"]
                )
                self.follower_current_lines_raw[joint_idx].set_data(
                    time_plot, arrays["follower_current_raw"]
                )
                self.follower_current_lines[joint_idx].set_data(
                    time_plot, arrays["follower_current"]
                )
                self.leader_force_rendering_damping_lines[joint_idx].set_data(
                    time_plot, arrays["leader_force_rendering_damping"]
                )
                all_lines.extend([
                    self.leader_current_lines[joint_idx],
                    self.leader_command_lines[joint_idx],
                    self.follower_current_lines_raw[joint_idx],
                    self.follower_current_lines[joint_idx],
                    self.leader_force_rendering_damping_lines[joint_idx],
                ])

            if "velocity" in self.col_map:
                self.leader_velocity_lines[joint_idx].set_data(
                    time_plot, arrays["leader_velocity"]
                )
                self.follower_velocity_lines[joint_idx].set_data(
                    time_plot, arrays["follower_velocity"]
                )
                all_lines.extend([
                    self.leader_velocity_lines[joint_idx],
                    self.follower_velocity_lines[joint_idx],
                ])

            if "position" in self.col_map:
                self.leader_position_lines[joint_idx].set_data(
                    time_plot, arrays["leader_position"]
                )
                self.follower_position_lines[joint_idx].set_data(
                    time_plot, arrays["follower_position"]
                )
                all_lines.extend([
                    self.leader_position_lines[joint_idx],
                    self.follower_position_lines[joint_idx],
                ])

            if "torque" in self.col_map:
                self.follower_torque_lines[joint_idx].set_data(
                    time_plot, arrays["follower_torque"]
                )
                self.follower_torque_lines_filtered[joint_idx].set_data(
                    time_plot, arrays["follower_torque_filtered"]
                )
                all_lines.extend([
                    self.follower_torque_lines[joint_idx],
                    self.follower_torque_lines_filtered[joint_idx],
                ])

            if should_scale and len(time_plot) > 0:
                xlim = (max(0, time_plot[-1] - 10), time_plot[-1] + 1)
                for ptype in self.plot_types:
                    self.axes[joint_idx, self.col_map[ptype]].set_xlim(*xlim)

        return all_lines

    def _trim_series(self, joint_idx: int, time_array: np.ndarray, plot_length: int):
        series = {
            "leader_current": np.array(list(self.leader_currents[joint_idx]), dtype=np.float32),
            "leader_command": np.array(list(self.leader_command_currents[joint_idx]), dtype=np.float32),
            "leader_force_rendering_damping": np.array(
                list(self.leader_force_rendering_damping_currents[joint_idx]),
                dtype=np.float32
            ),
            "follower_current": np.array(list(self.follower_currents[joint_idx]), dtype=np.float32),
            "follower_current_raw": np.array(
                list(self.follower_currents_raw[joint_idx]), dtype=np.float32
            ),
            "leader_velocity": np.array(list(self.leader_velocities[joint_idx]), dtype=np.float32),
            "follower_velocity": np.array(list(self.follower_velocities[joint_idx]), dtype=np.float32),
            "leader_position": np.array(list(self.leader_positions[joint_idx]), dtype=np.float32),
            "follower_position": np.array(list(self.follower_positions[joint_idx]), dtype=np.float32),
            "follower_torque": np.array(list(self.follower_torques[joint_idx]), dtype=np.float32),
            "follower_torque_filtered": np.array(
                list(self.follower_torques_filtered[joint_idx]), dtype=np.float32
            ),
        }

        if len(time_array) > plot_length:
            time_plot = time_array[-plot_length:]
            for key in series:
                series[key] = series[key][-plot_length:]
        else:
            time_plot = time_array

        min_len = min(len(time_plot), *(len(v) for v in series.values()))
        time_plot = time_plot[-min_len:]
        for key in series:
            series[key] = series[key][-min_len:]

        return time_plot, series

    def run(self) -> None:
        print("Starting Sharpa live plot")
        print(f"  Joints: {self.num_joints}")
        print(f"  Plotting: {', '.join(self.plot_types)}")
        print(f"  Leader motors: {self.leader_chain}")
        print(f"  Sharpa joints: {self.follower_labels}")
        print(f"  Force rendering thresholds (synth mA): +{self.thresholds_positive}, -{self.thresholds_negative}")
        print("\n  Close the plot window to stop.\n")

        self.teleop = SharpaDittoTeleop(self.config, state_queue=self.state_queue)
        self.teleop.connect()
        self.teleop.setup_motors()

        self.running = True
        control_thread = threading.Thread(
            target=self.teleop.start_control_loop, daemon=True
        )
        control_thread.start()

        data_thread = threading.Thread(target=self.data_collection_thread, daemon=True)
        data_thread.start()

        time.sleep(0.5)

        _ani = animation.FuncAnimation(
            self.fig,
            self.update_plots,
            interval=100,
            blit=True,
            cache_frame_data=False,
        )

        plt.show()

        print("\nStopping live plot...")
        self.running = False
        if self.teleop is not None:
            self.teleop.stop()
        control_thread.join(timeout=2.0)
        data_thread.join(timeout=1.0)
        if self.teleop is not None:
            self.teleop.disconnect()
        print("Live plot stopped.")


def main() -> None:
    plot_types = _parse_plot_flags()
    overrides = _build_overrides(sys.argv[1:])
    with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
        cfg = compose(config_name="config", overrides=overrides)

    plotter = SharpaLivePlotter(cfg, plot_types=plot_types)
    plotter.run()


if __name__ == "__main__":
    main()
