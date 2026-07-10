#!/usr/bin/env python3
"""Run Ditto tactile haptics: live loop, recording plot, or interactive tuning.

Requires a motor-keyed config YAML (see ``config/thumb.yaml`` and ``HAPTICS.md``).

Examples (from ``sharpa_teleop/ditto_haptics``)::

  python run_ditto_haptics.py config/thumb.yaml
  python run_ditto_haptics.py config/thumb.yaml --plot --duration 8
  python run_ditto_haptics.py config/thumb.yaml --tune
  python run_ditto_haptics.py config/thumb_index.yaml --tune --motor 1 # specify if only tunning one motor
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import select
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from enum import Enum, auto
from pathlib import Path

import matplotlib.pyplot as plt

_HAPTICS_DIR = Path(__file__).resolve().parent
_SHARPA_TELEOP = _HAPTICS_DIR.parent
_SHARPA_CONTROLLER = _SHARPA_TELEOP / "sharpa_controller"
for _path in (_SHARPA_CONTROLLER, _HAPTICS_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from ditto_haptics import (  # noqa: E402
    DittoHaptics,
    FzDirection,
    HapticsConfig,
    HapticsTrace,
    MotorHaptics,
    MotorHapticsConfig,
    level_for_plot,
)
from sharpa_hand.sharpa_hand import SharpaHand  # noqa: E402
from vibration_motors.vib_serial import VibMotor  # noqa: E402

DEFAULT_RATE = 40.0
PLOT_DURATION_S = 4.0
PLOT_WINDOW_S = 10.0
LIVE_PLOT_FIGSIZE = (14, 7)
IDENTIFY_LEVEL = 7
IDENTIFY_DURATION_S = 0.1
VIB_LEVEL_MAX = 10.0  # plot value for motor level 9 (OFF=0)
VIB_AXIS_TOP = 11.0  # headroom above max vib level on right axis


def _fz_ylim_aligned(rising: float, fz_at_max_level: float) -> tuple[float, float]:
    """Match Fz ``fz_at_max_level`` height to vib level ``VIB_LEVEL_MAX`` on twin axes."""
    span = max(fz_at_max_level - rising, 0.1)
    y_bottom = rising - span / 9.0
    y_top = y_bottom + (fz_at_max_level - y_bottom) * (VIB_AXIS_TOP / VIB_LEVEL_MAX)
    return y_bottom, y_top


def _apply_vib_axis(ax_vib) -> None:
    ax_vib.set_ylim(0, VIB_AXIS_TOP)
    ax_vib.set_yticks(range(int(VIB_LEVEL_MAX) + 1))



def _connect_hand(
    config: HapticsConfig,
    *,
    serial: str | None,
    calibrate: bool,
    verbose: bool,
) -> SharpaHand:
    hand = SharpaHand(serial=serial, enabled_joints=[], verbose=verbose)
    hand.connect()
    hand.configure()
    hand.start()
    hand.enable_tactile(config.tactile_enable_map())
    if calibrate:
        print("Calibrating tactile sensors...")
        if not hand.hand.calib_tactile():
            raise RuntimeError("Tactile calibration failed")
        print("Calibration complete.")
    return hand


def identify_motors(motor: VibMotor, motor_ids: list[int]) -> None:
    print("Identifying vibration motors...")
    for motor_id in motor_ids:
        print(f"MOTOR {motor_id}")
        motor.set_motor_level(motor_id, IDENTIFY_LEVEL)
        time.sleep(IDENTIFY_DURATION_S)
        motor.set_motor_level(motor_id, None)
        time.sleep(0.2)


def save_trace_csv(trace: HapticsTrace, path: Path, finger: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "fz_raw", "fz_filtered", "vib_plot", "active_min", "falling"])
        for row in zip(
            trace.times,
            trace.fz_raw,
            trace.fz,
            trace.level,
            trace.active_min,
            trace.falling,
            strict=True,
        ):
            writer.writerow(row)
    return path


def _cap_fz(fz: float, signal_max: float) -> float:
    return float("nan") if fz != fz else min(fz, signal_max)


def save_static_plot(
    trace: HapticsTrace,
    motor_cfg: MotorHapticsConfig,
    *,
    duration_s: float,
    output_path: Path | None,
    crop: bool,
) -> Path | None:
    plots_dir = _HAPTICS_DIR / "plots"
    png_path = output_path or plots_dir / (
        f"{motor_cfg.finger}_motor{motor_cfg.motor}_haptics_{datetime.now():%Y%m%d_%H%M%S}.png"
    )
    csv_path = png_path.with_suffix(".csv")

    rising_min = motor_cfg.rising_min
    falling_min = motor_cfg.falling_min
    signal_max = motor_cfg.signal_max

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(trace.times, trace.fz_raw, label="Fz (raw)", linewidth=1.0, color="tab:blue", alpha=0.35)
    if crop:
        fz_capped = [_cap_fz(f, signal_max) for f in trace.fz]
        ax.plot(trace.times, fz_capped, label=f"Fz filtered (capped {signal_max:g} N)", linewidth=1.8, color="tab:blue")
    else:
        ax.plot(trace.times, trace.fz, label="Fz (filtered)", linewidth=1.8, color="tab:blue")

    ax.axhline(rising_min, color="tab:blue", linestyle=":", alpha=0.55, label=f"rising_min ({rising_min:g} N)")
    ax.axhline(falling_min, color="tab:green", linestyle=":", alpha=0.55, label=f"falling_min ({falling_min:g} N)")
    ax.axhline(signal_max, color="tab:red", linestyle=":", alpha=0.65, label=f"signal_max ({signal_max:g} N)")
    ax.set_ylabel("Fz (N)")
    ax.grid(True, alpha=0.3)

    ax_vib = ax.twinx()
    ax_vib.plot(trace.times, trace.level, color="tab:orange", linewidth=1.5, label="vib")
    ax_vib.set_ylabel("vib (0=off, 1–10=levels 0–9)")
    _apply_vib_axis(ax_vib)

    y_bottom, y_top = _fz_ylim_aligned(rising_min, signal_max)
    if crop:
        ax.set_ylim(y_bottom, y_top)
    else:
        peak = max(
            (f for f in (*trace.fz_raw, *trace.fz) if f == f),
            default=signal_max,
        )
        ax.set_ylim(y_bottom, max(y_top, peak * 1.05))

    fall_times = [t for t, f in zip(trace.times, trace.falling, strict=True) if f > 0.5]
    if fall_times:
        ax.axvspan(min(fall_times), max(fall_times), color="tab:green", alpha=0.06)

    lines_l, labels_l = ax.get_legend_handles_labels()
    lines_r, labels_r = ax_vib.get_legend_handles_labels()
    ax.legend(lines_l + lines_r, labels_l + labels_r, loc="upper right", fontsize=8)
    ax.set_xlabel("time (s)")
    fig.suptitle(
        f"{motor_cfg.finger} motor {motor_cfg.motor} "
        f"(rising≥{rising_min:g}N, falling≥{falling_min:g}N) — {duration_s:.1f}s"
    )
    fig.tight_layout()
    png_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    save_trace_csv(trace, csv_path, motor_cfg.finger)
    print(f"Saved plot: {png_path}")
    print(f"Saved trace: {csv_path}")
    return png_path


@dataclass
class LiveView:
    lock: threading.Lock
    times: deque
    fz_raw: deque
    fz: deque
    vib: deque
    motor_id: int
    rising_min: float
    falling_min: float
    signal_max: float
    finger: str
    threshold_test: bool
    tuning_key: str
    status: str
    tune_mode: bool

    @classmethod
    def create(cls, motor_cfg: MotorHapticsConfig, *, tune_mode: bool = False) -> LiveView:
        return cls(
            lock=threading.Lock(),
            times=deque(),
            fz_raw=deque(),
            fz=deque(),
            vib=deque(),
            motor_id=motor_cfg.motor,
            rising_min=motor_cfg.rising_min,
            falling_min=motor_cfg.falling_min,
            signal_max=motor_cfg.signal_max,
            finger=motor_cfg.finger,
            threshold_test=False,
            tuning_key="rising_min",
            status="Running",
            tune_mode=tune_mode,
        )

    def reset_for_motor(self, motor_cfg: MotorHapticsConfig) -> None:
        with self.lock:
            self.times.clear()
            self.fz_raw.clear()
            self.fz.clear()
            self.vib.clear()
            self.motor_id = motor_cfg.motor
            self.finger = motor_cfg.finger
            self.rising_min = motor_cfg.rising_min
            self.falling_min = motor_cfg.falling_min
            self.signal_max = motor_cfg.signal_max
            self.threshold_test = False
            self.tuning_key = "rising_min"
            self.status = f"Tuning motor {motor_cfg.motor} ({motor_cfg.finger})"

    def set_thresholds(self, *, rising_min: float, falling_min: float, signal_max: float) -> None:
        with self.lock:
            self.rising_min = rising_min
            self.falling_min = falling_min
            self.signal_max = signal_max

    def append(self, t: float, sample) -> None:
        with self.lock:
            self.times.append(t)
            self.fz_raw.append(float("nan") if sample.fz_raw is None else sample.fz_raw)
            self.fz.append(float("nan") if sample.fz is None else sample.fz)
            self.vib.append(level_for_plot(sample.level))

    def trim(self, window_s: float) -> None:
        with self.lock:
            while self.times and self.times[-1] - self.times[0] > window_s:
                self.times.popleft()
                self.fz_raw.popleft()
                self.fz.popleft()
                self.vib.popleft()

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "times": list(self.times),
                "fz_raw": list(self.fz_raw),
                "fz": list(self.fz),
                "vib": list(self.vib),
                "motor_id": self.motor_id,
                "finger": self.finger,
                "rising_min": self.rising_min,
                "falling_min": self.falling_min,
                "signal_max": self.signal_max,
                "threshold_test": self.threshold_test,
                "tuning_key": self.tuning_key,
                "status": self.status,
                "tune_mode": self.tune_mode,
            }


class LivePlot:
    def __init__(self, window_s: float, title: str) -> None:
        import matplotlib.pyplot as plt

        self.plt = plt
        self.window_s = window_s
        self.fig, self.ax = plt.subplots(figsize=LIVE_PLOT_FIGSIZE)
        self.ax_vib = self.ax.twinx()
        self._fz_raw_line = None
        self._fz_line = None
        self._vib_line = None
        self._thresholds: list = []
        self.fig.canvas.manager.set_window_title(title)  # type: ignore[union-attr]

    def set_title(self, title: str) -> None:
        self.fig.canvas.manager.set_window_title(title)  # type: ignore[union-attr]
        self.ax.set_title(title)

    def reset(self) -> None:
        self._fz_raw_line = None
        self._fz_line = None
        self._vib_line = None
        for artist in self._thresholds:
            artist.remove()
        self._thresholds.clear()

    def redraw(self, snap: dict) -> None:
        times = snap["times"]
        if not times:
            return

        t_min = max(0.0, times[-1] - self.window_s)
        idx = next(i for i, t in enumerate(times) if t >= t_min)
        t_slice = times[idx:]
        fz_raw_slice = snap["fz_raw"][idx:]
        fz_slice = snap["fz"][idx:]
        vib_slice = snap["vib"][idx:]

        if self._fz_line is None:
            (self._fz_raw_line,) = self.ax.plot(
                t_slice, fz_raw_slice, color="tab:blue", linewidth=1.0, alpha=0.35, label="Fz (raw)"
            )
            (self._fz_line,) = self.ax.plot(
                t_slice, fz_slice, color="tab:blue", linewidth=1.8, label="Fz (filtered)"
            )
            (self._vib_line,) = self.ax_vib.plot(t_slice, vib_slice, color="tab:orange", linewidth=1.5, label="vib")
            self.ax.set_ylabel("Fz (N)")
            self.ax_vib.set_ylabel("vib (0=off, 1–10=levels 0–9)")
            _apply_vib_axis(self.ax_vib)
            self.ax.grid(True, alpha=0.3)
            self.ax.set_title(f"motor {snap['motor_id']} — {snap['finger']}")
        else:
            self._fz_raw_line.set_data(t_slice, fz_raw_slice)
            self._fz_line.set_data(t_slice, fz_slice)
            self._vib_line.set_data(t_slice, vib_slice)

        for artist in self._thresholds:
            artist.remove()
        self._thresholds.clear()

        rising = snap["rising_min"]
        smax = snap["signal_max"]
        falling = snap["falling_min"]
        tuning_key = snap["tuning_key"]
        threshold_test = snap["threshold_test"]
        tune_mode = snap["tune_mode"]

        def hline(y: float, color: str, label: str, *, active: bool = False) -> None:
            self._thresholds.append(
                self.ax.axhline(
                    y,
                    color=color,
                    linestyle=":",
                    linewidth=1.2,
                    alpha=0.85 if active else 0.55,
                    label=label,
                )
            )

        rising_label = (
            f"rising_min / level 9 ({rising:g} N)"
            if threshold_test and tune_mode
            else f"rising_min ({rising:g} N)"
        )
        hline(rising, "tab:blue", rising_label, active=tune_mode and tuning_key == "rising_min")
        if not (threshold_test and tune_mode):
            hline(smax, "tab:red", f"signal_max ({smax:g} N)", active=tune_mode and tuning_key == "signal_max")
        hline(falling, "tab:green", f"falling_min ({falling:g} N)", active=tune_mode and tuning_key == "falling_min")

        if threshold_test and tune_mode:
            peak = max(
                (f for f in (*fz_raw_slice, *fz_slice) if f == f),
                default=rising,
            )
            y_bottom, y_top = _fz_ylim_aligned(rising, rising)
            y_top = max(y_top, peak * 1.05, rising + 0.3)
        else:
            y_bottom, y_top = _fz_ylim_aligned(rising, smax)
        _apply_vib_axis(self.ax_vib)
        self.ax.set_xlim(t_min, max(t_min + 1.0, times[-1] + 0.05))
        self.ax.set_ylim(y_bottom, y_top)
        self.ax.set_xlabel("time (s)")
        self.fig.suptitle(snap["status"])
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def _refresh_plot(plot: LivePlot | None, live: LiveView | None) -> None:
    if plot is None or live is None:
        return
    plot.redraw(live.snapshot())
    plot.plt.pause(0.001)


def _run_plot_refresh(plot: LivePlot, live: LiveView, stop: threading.Event) -> None:
    while not stop.is_set():
        _refresh_plot(plot, live)
        plot.plt.pause(0.05)


def _run_loop(
    *,
    haptics: DittoHaptics | None = None,
    motor_haptics: MotorHaptics | None = None,
    display_motor: int,
    rate: float,
    duration_s: float | None,
    trace: HapticsTrace | None,
    live: LiveView | None,
    quiet: bool,
    use_config_falling: bool,
    stop: threading.Event | None = None,
) -> None:
    if (haptics is None) == (motor_haptics is None):
        raise ValueError("provide exactly one of haptics or motor_haptics")

    period = 1.0 / rate if rate > 0 else 0.0
    t0 = time.perf_counter()
    end = t0 + duration_s if duration_s is not None else None

    while stop is None or not stop.is_set():
        if end is not None and time.perf_counter() >= end:
            break
        loop_start = time.perf_counter()
        if motor_haptics is not None:
            sample = motor_haptics.update(use_config_falling=use_config_falling)
            samples = {motor_haptics.config.motor: sample}
        else:
            samples = haptics.update(use_config_falling=use_config_falling)  # type: ignore[union-attr]
        sample = samples[display_motor]
        t = loop_start - t0

        if trace is not None:
            trace.append(t, sample)
        if live is not None:
            live.append(t, samples[live.motor_id])
            live.trim(PLOT_WINDOW_S)

        if not quiet:
            raw_str = "no frame" if sample.fz_raw is None else f"{sample.fz_raw:5.3f}"
            fz_str = "no frame" if sample.fz is None else f"{sample.fz:5.3f}"
            bound = "falling" if sample.direction is FzDirection.FALLING else "rising"
            motor_bits = []
            for motor_id, motor_sample in sorted(samples.items()):
                lvl_str = "off" if motor_sample.level is None else str(motor_sample.level)
                motor_bits.append(f"m{motor_id}={lvl_str}")
            print(
                f"Fz raw={raw_str} filt={fz_str} | {' '.join(motor_bits)} | "
                f"{bound} (min={sample.active_min:.2f})"
            )

        if period > 0:
            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


# ---- tuning ----

class TunePhase(Enum):
    RISING_MIN = auto()
    SIGNAL_MAX = auto()
    FALLING_MIN = auto()


def _read_line(prompt: str, plot: LivePlot | None, live: LiveView | None) -> str:
    print(prompt, end="", flush=True)
    if plot is None or live is None:
        return input().strip()
    while True:
        _refresh_plot(plot, live)
        ready, _, _ = select.select([sys.stdin], [], [], 0.05)
        if ready:
            return sys.stdin.readline().strip()


def _prompt_float(
    label: str,
    current: float,
    *,
    plot: LivePlot | None = None,
    live: LiveView | None = None,
) -> float:
    while True:
        raw = _read_line(f"{label} [{current:g} N]: ", plot, live)
        if raw == "":
            return current
        try:
            return float(raw)
        except ValueError:
            print("  Enter a number or press Enter to keep the current value.")


def _confirm_keep(*, plot: LivePlot | None = None, live: LiveView | None = None) -> bool:
    while True:
        ans = _read_line("Keep this value? [y/n]: ", plot, live).lower()
        if ans in ("y", "yes", ""):
            return True
        if ans in ("n", "no"):
            return False


def _tune_one(
    haptics,
    live: LiveView | None,
    *,
    plot: LivePlot | None = None,
    key,
    label,
    current,
    setter,
    validator=None,
) -> float:
    if live is not None:
        live.tuning_key = key
        live.status = f"Tuning {label} — type a value and press Enter to try it."
    value = current
    while True:
        print(f"\n--- {label} ---")
        trial = _prompt_float(label, value, plot=plot, live=live)
        if validator is not None:
            try:
                validator(trial)
            except ValueError as exc:
                print(f"  {exc}")
                continue
        setter(trial)
        haptics.reset_hyst()
        print("Feel it out, then:")
        if _confirm_keep(plot=plot, live=live):
            if live is not None:
                live.status = f"Locked {label} = {trial:g} N"
            return trial
        value = trial


def _validate_nonneg(v: float) -> None:
    if v < 0:
        raise ValueError("must be >= 0")


def _validate_signal_max(v: float, rising_min: float) -> None:
    if v <= rising_min:
        raise ValueError(f"must be greater than rising_min ({rising_min:g} N)")


def _validate_falling_min(v: float, rising_min: float, signal_max: float) -> None:
    if v <= rising_min:
        raise ValueError(f"must be greater than rising_min ({rising_min:g} N)")
    if v >= signal_max:
        raise ValueError(f"must be less than signal_max ({signal_max:g} N)")


def run_tune(
    motor_haptics: MotorHaptics,
    motor_cfg: MotorHapticsConfig,
    live: LiveView | None,
    *,
    plot: LivePlot | None = None,
) -> MotorHapticsConfig:
    tuned = MotorHapticsConfig(
        motor=motor_cfg.motor,
        finger=motor_cfg.finger,
        tactile_channel=motor_cfg.tactile_channel,
        rising_min=motor_cfg.rising_min,
        falling_min=motor_cfg.falling_min,
        signal_max=motor_cfg.signal_max,
        direction_eps=motor_cfg.direction_eps,
        direction_window=motor_cfg.direction_window,
        fz_tau_s=motor_cfg.fz_tau_s,
    )

    def sync_live() -> None:
        if live is None:
            return
        live.set_thresholds(
            rising_min=motor_haptics.rising_min,
            falling_min=motor_haptics.falling_min,
            signal_max=motor_haptics.signal_max,
        )

    def set_status(msg: str) -> None:
        if live is not None:
            live.status = msg

    print(
        f"\nTuning motor {motor_cfg.motor} ({motor_cfg.finger}, "
        f"tactile ch {motor_cfg.tactile_channel})"
    )
    print("Empty input keeps the current/config value.\n")

    motor_haptics.apply_config(tuned)

    if live is not None:
        live.threshold_test = True
    motor_haptics.set_tuning_mode(threshold_test=True)
    motor_haptics.signal_max = tuned.rising_min
    sync_live()
    set_status("Tuning rising_min — full vib (level 9) at/above threshold.")

    tuned.rising_min = _tune_one(
        motor_haptics,
        live,
        plot=plot,
        key="rising_min",
        label="rising_min",
        current=tuned.rising_min,
        setter=lambda v: (
            setattr(motor_haptics, "rising_min", v),
            setattr(motor_haptics, "signal_max", v),
            sync_live(),
        ),
        validator=_validate_nonneg,
    )
    motor_haptics.rising_min = tuned.rising_min

    if live is not None:
        live.threshold_test = False
    motor_haptics.set_tuning_mode(threshold_test=False)
    motor_haptics.signal_max = tuned.signal_max
    sync_live()
    set_status("Tuning signal_max — graded vib ramp.")

    tuned.signal_max = _tune_one(
        motor_haptics,
        live,
        plot=plot,
        key="signal_max",
        label="signal_max (level 9)",
        current=tuned.signal_max,
        setter=lambda v: (setattr(motor_haptics, "signal_max", v), sync_live()),
        validator=lambda v: _validate_signal_max(v, tuned.rising_min),
    )

    motor_haptics.falling_min = tuned.falling_min
    sync_live()
    set_status("Tuning falling_min.")

    tuned.falling_min = _tune_one(
        motor_haptics,
        live,
        plot=plot,
        key="falling_min",
        label="falling_min",
        current=tuned.falling_min,
        setter=lambda v: (setattr(motor_haptics, "falling_min", v), sync_live()),
        validator=lambda v: _validate_falling_min(v, tuned.rising_min, tuned.signal_max),
    )

    tuned.validate()
    return tuned


def _resolve_motor_id(config: HapticsConfig, motor: int | None) -> int:
    motor_ids = config.motor_ids()
    if motor is None:
        return motor_ids[0]
    if motor not in config.motors:
        raise ValueError(f"motor {motor} not in config (available: {motor_ids})")
    return motor


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="Motor-keyed config YAML (e.g. config/thumb.yaml)")
    parser.add_argument("--motor", type=int, help="Limit --tune or --plot to one motor id (default: all motors for --tune)")
    parser.add_argument("--vib-port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--sharpa-serial")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--skip-identify", action="store_true")
    parser.add_argument("--tune", action="store_true", help="Interactive tuning wizard")
    parser.add_argument("--plot", action="store_true", help="Record trace and save static plot")
    parser.add_argument("--duration", type=float, default=PLOT_DURATION_S)
    parser.add_argument("--plot-out", type=Path)
    parser.add_argument("--no-crop", action="store_true")
    parser.add_argument("--no-live-plot", action="store_true", help="Disable live matplotlib window")
    parser.add_argument("--wait-enter", action="store_true")
    args = parser.parse_args()

    if args.tune and args.plot:
        parser.error("use either --tune or --plot, not both")
    if args.duration <= 0:
        parser.error("duration must be positive")

    config_path = args.config if args.config.is_absolute() else _HAPTICS_DIR / args.config
    if not config_path.exists():
        parser.error(f"config not found: {config_path}")

    config = HapticsConfig.load(config_path)
    try:
        display_motor = _resolve_motor_id(config, args.motor)
    except ValueError as exc:
        parser.error(str(exc))
    motor_cfg = config.motors[display_motor]
    motor_ids = config.motor_ids()
    use_gui = (not args.no_live_plot) and (not args.plot or args.tune)

    if args.plot:
        args.quiet = True
        args.wait_enter = True

    hand = None
    vib = None
    try:
        hand = _connect_hand(config, serial=args.sharpa_serial, calibrate=args.calibrate, verbose=not args.quiet)
        vib = VibMotor(args.vib_port, baud=args.baud)
        haptics = DittoHaptics.from_sharpa_hand(config, vib, hand)

        if not args.skip_identify and not args.tune:
            identify_motors(vib, motor_ids)

        if args.wait_enter or args.plot:
            msg = f"Press Enter to start {args.duration:.0f}s capture..." if args.plot else "Press Enter to start..."
            input(msg)

        if args.tune:
            import matplotlib

            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt

            tune_motor_ids = motor_ids if args.motor is None else [display_motor]
            plot = None
            live = None

            if not args.no_live_plot:
                first_cfg = config.motors[tune_motor_ids[0]]
                live = LiveView.create(first_cfg, tune_mode=True)
                plot = LivePlot(PLOT_WINDOW_S, f"Tune motor {first_cfg.motor} ({first_cfg.finger})")
                plt.show(block=False)

            tuned_motors: list[tuple[int, MotorHapticsConfig]] = []
            for motor_id in tune_motor_ids:
                motor_cfg = config.motors[motor_id]
                tune_channel = haptics.motor(motor_id)

                if live is not None and plot is not None:
                    live.reset_for_motor(motor_cfg)
                    plot.reset()
                    plot.set_title(f"motor {motor_id} — {motor_cfg.finger}")

                loop_stop = threading.Event()
                threading.Thread(
                    target=_run_loop,
                    kwargs={
                        "motor_haptics": tune_channel,
                        "display_motor": motor_id,
                        "rate": args.rate,
                        "duration_s": None,
                        "trace": None,
                        "live": live,
                        "quiet": True,
                        "use_config_falling": True,
                        "stop": loop_stop,
                    },
                    daemon=True,
                ).start()

                tuned = run_tune(tune_channel, motor_cfg, live, plot=plot)
                loop_stop.set()
                tune_channel.stop()
                config.replace_motor(tuned)
                tuned_motors.append((motor_id, tuned))

            haptics.stop()
            out = HapticsConfig.tuned_path(config_path)
            config.save(out)
            print(f"\nSaved tuned config: {out}")
            for motor_id, tuned in tuned_motors:
                print(
                    f"  motor {motor_id}: rising_min={tuned.rising_min:g}  "
                    f"signal_max={tuned.signal_max:g}  falling_min={tuned.falling_min:g} N"
                )
            plt.close("all")
            return

        trace = HapticsTrace() if args.plot else None
        plot_motor_id = config.plot_motor
        use_live_plot = plot_motor_id is not None and not args.plot and not args.no_live_plot
        if not args.plot and not args.no_live_plot and plot_motor_id is None:
            print("No live plot: set plot_motor in config to a motor id.")
        live = (
            LiveView.create(config.motors[plot_motor_id])
            if use_live_plot
            else None
        )
        stop = threading.Event()
        plot = None

        if live is not None:
            import matplotlib

            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt

            plot_cfg = config.motors[plot_motor_id]
            plot = LivePlot(PLOT_WINDOW_S, f"Ditto haptics — motor {plot_motor_id} ({plot_cfg.finger})")
            plt.show(block=False)

            threading.Thread(
                target=_run_loop,
                kwargs={
                    "haptics": haptics,
                    "display_motor": display_motor,
                    "rate": args.rate,
                    "duration_s": None,
                    "trace": None,
                    "live": live,
                    "quiet": args.quiet,
                    "use_config_falling": True,
                    "stop": stop,
                },
                daemon=True,
            ).start()
            try:
                _run_plot_refresh(plot, live, stop)
            except KeyboardInterrupt:
                stop.set()
                raise
        else:
            _run_loop(
                haptics=haptics,
                display_motor=display_motor,
                rate=args.rate,
                duration_s=args.duration if args.plot else None,
                trace=trace,
                live=None,
                quiet=args.quiet,
                use_config_falling=True,
            )

        stop.set()
        if args.plot and trace is not None:
            save_static_plot(
                trace,
                motor_cfg,
                duration_s=args.duration,
                output_path=args.plot_out,
                crop=not args.no_crop,
            )
        if plot is not None:
            plt.close("all")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if vib is not None:
            vib.close()
        if hand is not None:
            hand.disconnect()


if __name__ == "__main__":
    main()
