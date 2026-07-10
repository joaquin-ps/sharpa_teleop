#!/usr/bin/env python3
"""Live plot for retargeting force rendering (index finger).

Compares, per leader index joint:
  - estimated external joint torque (Nm) from our Jᵀ·F  ── the "follower"
  - the current we actually render to the leader (mA)    ── the "inverse"

Columns:
  torque       : estimated Ditto joint torque (Nm), raw vs filtered
  current      : synthetic follower current (from estimated torque) vs rendered
                 leader command vs measured leader current (mA), with deadband
  force render : the force-rendering current contribution only, autoscaled on its
                 own axis so the (often small) haptic signal stays readable

Which finger(s) appear is inferred from the leader motors in the hand_config
(default ``ditto_2f_tactile``).

Usage (from sharpa_teleop repo root):
    python retargeting_teleop/viz/force_plot.py
    python retargeting_teleop/viz/force_plot.py hand_config=ditto_3f_tactile
    python retargeting_teleop/viz/force_plot.py u2d2.usb_port=/dev/ttyUSB0

Control mode is config-only (``hand_config.control.fingers``). The Sharpa
follower is always connected (same as ``run_force_render.py``).

Raise gains at runtime, e.g.:
    ... hand_config.leader.joint_settings.1.current_control.force_rendering_gain=0.05
"""

from __future__ import annotations

import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

_PKG = Path(__file__).resolve().parent.parent
_REPO = _PKG.parent
for _path in (_PKG, _REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import matplotlib.animation as animation  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from hydra import compose, initialize_config_dir  # noqa: E402

from _paths import CONF_DIR, DITTO_CONF_DIR  # noqa: E402
from retargeting.paths import (  # noqa: E402
    DITTO_3F_LEADER_JOINT_NAMES,
    DITTO_3F_LEADER_MOTOR_IDS,
    DITTO_LEADER_JOINT_NAMES,
    DITTO_LEADER_MOTOR_IDS,
)
from teleop.force_render import (  # noqa: E402
    RetargetForceRenderTeleop,
    finger_modes_from_config,
)

_MOTOR_TO_JOINT = dict(zip(DITTO_3F_LEADER_MOTOR_IDS, DITTO_3F_LEADER_JOINT_NAMES))

DEFAULT_HAND_CONFIG = "ditto_2f_tactile"


@dataclass
class RunFlags:
    retarget_hz: float = 40.0


def _strip_run_flags() -> RunFlags:
    flags = RunFlags()
    remaining = [sys.argv[0]]
    args = iter(sys.argv[1:])
    for arg in args:
        if arg == "--retarget-rate":
            flags.retarget_hz = float(next(args))
        elif arg.startswith("--retarget-rate="):
            flags.retarget_hz = float(arg.split("=", 1)[1])
        else:
            remaining.append(arg)
    sys.argv = remaining
    return flags


class ForceRenderPlotter:
    """Live plotter for RetargetForceRenderTeleop (torque + current columns)."""

    def __init__(self, config, flags: RunFlags, max_history: int = 4000):
        self.config = config
        self.flags = flags
        self.max_history = max_history
        self.plot_window_seconds = 10.0

        self.finger_modes = finger_modes_from_config(config)

        self.leader_chain = list(config.hand_config.leader.motor_ids)
        self.num_joints = len(self.leader_chain)
        self.labels = [
            f"M{m} ({_MOTOR_TO_JOINT.get(m, '?')})" for m in self.leader_chain
        ]
        # Rows to actually draw: chain indices whose joint has plot_joint != false.
        self.plot_indices = self._plot_indices()
        self.num_rows = len(self.plot_indices)

        self.thresholds_positive, self.thresholds_negative = self._read_thresholds()

        n = self.num_joints
        self.time_data: deque[float] = deque(maxlen=max_history)
        self.tau_raw = [deque(maxlen=max_history) for _ in range(n)]
        self.tau_filt = [deque(maxlen=max_history) for _ in range(n)]
        self.synth_raw = [deque(maxlen=max_history) for _ in range(n)]
        self.synth_filt = [deque(maxlen=max_history) for _ in range(n)]
        self.leader_cmd = [deque(maxlen=max_history) for _ in range(n)]
        self.leader_meas = [deque(maxlen=max_history) for _ in range(n)]
        self.damping = [deque(maxlen=max_history) for _ in range(n)]
        self.force_render = [deque(maxlen=max_history) for _ in range(n)]

        self.state_queue: queue.Queue = queue.Queue(maxsize=50)
        self.running = False
        self.controller: RetargetForceRenderTeleop | None = None

        self._build_controller()
        self._setup_plots()

    def _plot_indices(self) -> list[int]:
        """Chain indices to draw: leader joints with plot_joint != false (default true)."""
        flags: dict[int, bool] = {}
        leader_cfg = self.config.hand_config.leader
        if leader_cfg.get("joint_settings"):
            for jc in leader_cfg.joint_settings:
                mid = jc.get("motor_id")
                if mid is not None:
                    flags[mid] = bool(jc.get("plot_joint", True))
        idxs = [i for i, m in enumerate(self.leader_chain) if flags.get(m, True)]
        if not idxs:  # nothing flagged → plot all rather than draw an empty figure
            print("  [plot] no joints have plot_joint=true; plotting all")
            return list(range(self.num_joints))
        skipped = [m for i, m in enumerate(self.leader_chain) if i not in idxs]
        if skipped:
            print(f"  [plot] skipping joints (plot_joint=false): {skipped}")
        return idxs

    def _read_thresholds(self) -> tuple[list[float], list[float]]:
        pos_map, neg_map = {}, {}
        leader_cfg = self.config.hand_config.leader
        if leader_cfg.get("joint_settings"):
            for jc in leader_cfg.joint_settings:
                mid = jc.get("motor_id")
                cc = jc.get("current_control")
                if mid is not None and cc is not None:
                    base = cc.get("force_rendering_threshold", 50.0)
                    pos_map[mid] = cc.get("force_rendering_threshold_positive", base)
                    neg_map[mid] = cc.get("force_rendering_threshold_negative", base)
        pos = [float(pos_map.get(m, 50.0)) for m in self.leader_chain]
        neg = [float(neg_map.get(m, 50.0)) for m in self.leader_chain]
        return pos, neg

    def _build_controller(self) -> None:
        from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

        self.controller = RetargetForceRenderTeleop(
            self.config,
            sharpa_follower=SharpaFollowerSession(self.config, verbose=True),
            retarget_hz=self.flags.retarget_hz,
            state_queue=self.state_queue,
        )

    def _setup_plots(self) -> None:
        self.fig, axes = plt.subplots(
            self.num_rows, 3, figsize=(16, 3.2 * self.num_rows), squeeze=False
        )
        self.axes = axes
        self.tau_raw_lines, self.tau_filt_lines = [], []
        self.synth_raw_lines, self.synth_filt_lines = [], []
        self.cmd_lines, self.meas_lines, self.damp_lines = [], [], []
        self.fr_lines, self.fr_damp_lines, self.fr_net_lines = [], [], []

        # row = plot row, j = index into the full leader chain / data deques.
        for row, j in enumerate(self.plot_indices):
            ax_t = self.axes[row, 0]
            ax_t.set_title(f"{self.labels[j]} — estimated joint torque (Nm)")
            ax_t.set_ylabel("Torque (Nm)")
            ax_t.grid(True, alpha=0.3)
            ax_t.axhline(y=0, color="k", linestyle="-", alpha=0.3)
            lr, = ax_t.plot([], [], "m-", label="τ̂ raw", linewidth=1.2, alpha=0.7)
            lf, = ax_t.plot([], [], color="orange", linestyle="--", label="τ̂ filt", linewidth=1.5)
            self.tau_raw_lines.append(lr)
            self.tau_filt_lines.append(lf)

            ax_c = self.axes[row, 1]
            ax_c.set_title(f"{self.labels[j]} — current (mA): follower vs rendered")
            ax_c.set_ylabel("Current (mA)")
            ax_c.grid(True, alpha=0.3)
            ax_c.axhline(y=self.thresholds_positive[j], color="r", linestyle="--", alpha=0.6)
            ax_c.axhline(y=-self.thresholds_negative[j], color="r", linestyle="--", alpha=0.6)
            ax_c.set_ylim(
                -(self.thresholds_negative[j] + 100),
                self.thresholds_positive[j] + 100,
            )
            sr, = ax_c.plot([], [], "g-", label="follower raw", linewidth=1.0, alpha=0.6)
            sf, = ax_c.plot([], [], color="darkgreen", linestyle="--", label="follower filt", linewidth=1.3)
            cmd, = ax_c.plot([], [], "c-", label="leader cmd (rendered)", linewidth=1.6)
            meas, = ax_c.plot([], [], "b-", label="leader meas", linewidth=1.0, alpha=0.7)
            damp, = ax_c.plot([], [], color="purple", linestyle=":", label="damping", linewidth=1.2)
            self.synth_raw_lines.append(sr)
            self.synth_filt_lines.append(sf)
            self.cmd_lines.append(cmd)
            self.meas_lines.append(meas)
            self.damp_lines.append(damp)

            # Dedicated column: the rendered current contributions, autoscaled on
            # their own so the (often small) haptic signals are actually visible:
            # force rendering, damping, and the net command sent to the motors.
            ax_f = self.axes[row, 2]
            ax_f.set_title(f"{self.labels[j]} — rendered current contributions (mA)")
            ax_f.set_ylabel("Current (mA)")
            ax_f.grid(True, alpha=0.3)
            ax_f.axhline(y=0, color="k", linestyle="-", alpha=0.3)
            fr, = ax_f.plot([], [], color="crimson", label="force render", linewidth=1.6)
            frd, = ax_f.plot([], [], color="purple", linestyle=":", label="damping", linewidth=1.4)
            frn, = ax_f.plot([], [], color="black", linestyle="--", label="net (sent)", linewidth=1.2, alpha=0.8)
            self.fr_lines.append(fr)
            self.fr_damp_lines.append(frd)
            self.fr_net_lines.append(frn)

            if row == self.num_rows - 1:
                ax_t.set_xlabel("Time (s)")
                ax_c.set_xlabel("Time (s)")
                ax_f.set_xlabel("Time (s)")

        self.axes[0, 0].legend(loc="upper right", fontsize=8)
        self.axes[0, 1].legend(loc="upper right", fontsize=8)
        self.axes[0, 2].legend(loc="upper right", fontsize=8)
        plt.tight_layout()

    def _data_thread(self) -> None:
        start = time.time()
        while self.running:
            try:
                state = self.state_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            t = time.time() - start
            self.time_data.append(t)
            ld = state.get("leader", {})
            fo = state.get("follower", {})
            for j in range(self.num_joints):
                self._push(self.tau_raw[j], fo.get("torques_nm", []), j)
                self._push(self.tau_filt[j], fo.get("torques_nm_filtered", []), j)
                self._push(self.synth_raw[j], fo.get("currents_raw", []), j)
                self._push(self.synth_filt[j], fo.get("currents", []), j)
                self._push(self.leader_cmd[j], ld.get("command_currents", []), j)
                self._push(self.leader_meas[j], ld.get("currents", []), j)
                self._push(self.damping[j], ld.get("force_rendering_damping_currents", []), j)
                self._push(self.force_render[j], ld.get("force_rendering_currents", []), j)

    @staticmethod
    def _push(dq: deque, values: list, idx: int) -> None:
        dq.append(values[idx] if idx < len(values) else 0.0)

    def _update(self, _frame):
        if not self.running or len(self.time_data) == 0:
            return []
        t = np.array(self.time_data, dtype=np.float32)
        lines = []
        # row = plot row, j = index into the full leader chain / data deques.
        for row, j in enumerate(self.plot_indices):
            n = min(
                len(t), len(self.tau_raw[j]), len(self.synth_raw[j]),
                len(self.leader_cmd[j]), len(self.leader_meas[j]),
            )
            if n == 0:
                continue
            tt = t[-n:]
            pairs = [
                (self.tau_raw_lines[row], self.tau_raw[j]),
                (self.tau_filt_lines[row], self.tau_filt[j]),
                (self.synth_raw_lines[row], self.synth_raw[j]),
                (self.synth_filt_lines[row], self.synth_filt[j]),
                (self.cmd_lines[row], self.leader_cmd[j]),
                (self.meas_lines[row], self.leader_meas[j]),
                (self.damp_lines[row], self.damping[j]),
                (self.fr_lines[row], self.force_render[j]),
                (self.fr_damp_lines[row], self.damping[j]),
                (self.fr_net_lines[row], self.leader_cmd[j]),
            ]
            for line, dq in pairs:
                line.set_data(tt, np.array(dq, dtype=np.float32)[-n:])
                lines.append(line)
            xlim = (max(0.0, float(tt[-1]) - self.plot_window_seconds), float(tt[-1]) + 1.0)
            for col in range(3):
                self.axes[row, col].set_xlim(*xlim)
                if col == 1:
                    # Current column: fixed y-range = threshold ± 100 mA (deadband visible).
                    self.axes[row, col].set_ylim(
                        -(self.thresholds_negative[j] + 100),
                        self.thresholds_positive[j] + 100,
                    )
                else:
                    self.axes[row, col].relim()
                    self.axes[row, col].autoscale_view(scalex=False, scaley=True)
        return lines

    def run(self) -> None:
        assert self.controller is not None
        print("Starting force-render live plot")
        print(f"  Leader motors: {self.leader_chain}")
        print(f"  Sharpa: on")
        modes_str = ", ".join(
            f"{f} (pos:{m['position']}, force:{m['force']})"
            if isinstance(m["force"], dict)
            else f"{f} (pos:{m['position']}, force:{m['force']})"
            for f, m in self.finger_modes.items()
        )
        print(f"  Per-finger mode: {modes_str}")
        print("  Close the plot window to stop.\n")

        self.controller.connect()
        self.controller.setup_motors()
        self.running = True

        control_thread = threading.Thread(
            target=self.controller.start_control_loop, daemon=True
        )
        control_thread.start()
        data_thread = threading.Thread(target=self._data_thread, daemon=True)
        data_thread.start()
        time.sleep(0.5)

        _ani = animation.FuncAnimation(
            self.fig, self._update, interval=100, blit=False, cache_frame_data=False
        )
        plt.show()

        print("\nStopping force-render plot...")
        self.running = False
        self.controller.stop()
        control_thread.join(timeout=2.0)
        data_thread.join(timeout=1.0)
        self.controller.disconnect()
        print("Stopped.")


def main() -> None:
    flags = _strip_run_flags()
    searchpath = f"hydra.searchpath=[file://{DITTO_CONF_DIR}]"
    overrides = [searchpath]
    if not any(arg.startswith("hand_config=") for arg in sys.argv[1:]):
        overrides.append(f"hand_config={DEFAULT_HAND_CONFIG}")
    overrides += sys.argv[1:]
    with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
        cfg = compose(config_name="config", overrides=overrides)

    ForceRenderPlotter(cfg, flags).run()


if __name__ == "__main__":
    main()
