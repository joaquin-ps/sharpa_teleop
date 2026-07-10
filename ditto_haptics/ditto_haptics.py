"""Ditto haptics: tactile Fz -> vibration mapping.

Standalone library for embedding in teleop loops (e.g. force render). Does not
connect to hardware — callers provide a SharpaHand (with tactile already enabled)
or any ``read_fz`` callable returning normal force in newtons.

Example (host owns SharpaHand lifecycle)::

    from ditto_haptics import DittoHaptics, HapticsConfig, read_fz

    cfg = HapticsConfig.load("sharpa_teleop/ditto_haptics/config/thumb.yaml")
    hand.enable_tactile(cfg.tactile_enable_map())
    haptics = DittoHaptics.from_sharpa_hand(cfg, vib_motor, hand)

    while running:
        haptics.update()
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable, Protocol

import yaml

DIRECTION_EPS_DEFAULT = 0.05  # min |dFz/dt| (N/s) over window to switch rising/falling
DIRECTION_WINDOW_DEFAULT = 3  # samples in sliding window for average slope
FZ_TAU_S_DEFAULT = 0.05  # low-pass time constant (s); 0 = off


class FzDirection(Enum):
    RISING = "rising"
    FALLING = "falling"


class VibDriver(Protocol):
    def set_motor_level(self, motor: int, level: int | None) -> None: ...


@dataclass
class MotorHapticsConfig:
    """Per-motor tactile → vibration mapping."""

    motor: int
    finger: str
    tactile_channel: int
    rising_min: float
    falling_min: float
    signal_max: float
    direction_eps: float = DIRECTION_EPS_DEFAULT
    direction_window: int = DIRECTION_WINDOW_DEFAULT
    fz_tau_s: float = FZ_TAU_S_DEFAULT

    def validate(self) -> None:
        if self.rising_min >= self.falling_min:
            raise ValueError("rising_min must be less than falling_min")
        if self.falling_min >= self.signal_max:
            raise ValueError("falling_min must be less than signal_max")
        if self.direction_eps < 0:
            raise ValueError("direction_eps must be >= 0")
        if self.direction_window < 2:
            raise ValueError("direction_window must be at least 2")
        if self.fz_tau_s < 0:
            raise ValueError("fz_tau_s must be >= 0")

    def to_dict(self) -> dict:
        return {
            "finger": self.finger,
            "tactile_channel": self.tactile_channel,
            "rising_min": self.rising_min,
            "falling_min": self.falling_min,
            "signal_max": self.signal_max,
            "direction_eps": self.direction_eps,
            "direction_window": self.direction_window,
            "fz_tau_s": self.fz_tau_s,
        }

    @classmethod
    def from_dict(cls, motor: int, data: dict) -> MotorHapticsConfig:
        cfg = cls(
            motor=motor,
            finger=str(data["finger"]),
            tactile_channel=int(data["tactile_channel"]),
            rising_min=float(data["rising_min"]),
            falling_min=float(data["falling_min"]),
            signal_max=float(data["signal_max"]),
            direction_eps=float(data.get("direction_eps", DIRECTION_EPS_DEFAULT)),
            direction_window=int(data.get("direction_window", DIRECTION_WINDOW_DEFAULT)),
            fz_tau_s=float(data.get("fz_tau_s", FZ_TAU_S_DEFAULT)),
        )
        cfg.validate()
        return cfg


@dataclass
class HapticsConfig:
    """Motor-keyed haptics config (one YAML, many motors)."""

    motors: dict[int, MotorHapticsConfig]
    plot_motor: int | None = None

    def validate(self) -> None:
        if not self.motors:
            raise ValueError("config must define at least one motor")
        for cfg in self.motors.values():
            cfg.validate()
        if self.plot_motor is not None and self.plot_motor not in self.motors:
            raise ValueError(f"plot_motor {self.plot_motor} is not defined under motors")

    def motor_ids(self) -> list[int]:
        return sorted(self.motors.keys())

    def tactile_enable_map(self) -> dict[str, int]:
        return {cfg.finger: cfg.tactile_channel for cfg in self.motors.values()}

    @classmethod
    def load(cls, path: Path | str) -> HapticsConfig:
        path = Path(path)
        data = yaml.safe_load(path.read_text()) or {}
        motors_raw = data.get("motors")
        if motors_raw is None:
            raise ValueError("config must have a top-level 'motors' mapping")
        motors: dict[int, MotorHapticsConfig] = {}
        for key, entry in motors_raw.items():
            motor_id = int(key)
            if motor_id in motors:
                raise ValueError(f"duplicate motor id: {motor_id}")
            motors[motor_id] = MotorHapticsConfig.from_dict(motor_id, entry)
        plot_motor_raw = data.get("plot_motor")
        plot_motor = None if plot_motor_raw is None else int(plot_motor_raw)
        cfg = cls(motors=motors, plot_motor=plot_motor)
        cfg.validate()
        return cfg

    def save(self, path: Path | str) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload: dict = {
            "motors": {motor_id: cfg.to_dict() for motor_id, cfg in sorted(self.motors.items())},
        }
        if self.plot_motor is not None:
            payload["plot_motor"] = self.plot_motor
        path.write_text(yaml.safe_dump(payload, sort_keys=False))
        return path

    def replace_motor(self, motor_cfg: MotorHapticsConfig) -> None:
        motor_cfg.validate()
        self.motors[motor_cfg.motor] = motor_cfg

    @staticmethod
    def tuned_path(config_path: Path | str) -> Path:
        path = Path(config_path)
        return path.with_name(f"{path.stem}_tune{path.suffix}")


# Back-compat alias (one motor entry in a HapticsConfig file).
FingerHapticsConfig = MotorHapticsConfig


@dataclass
class HapticsSample:
    fz_raw: float | None
    fz: float | None  # low-pass filtered; used for mapping
    level: int | None
    active_min: float
    direction: FzDirection


@dataclass
class HapticsTrace:
    times: list[float] = field(default_factory=list)
    fz_raw: list[float] = field(default_factory=list)
    fz: list[float] = field(default_factory=list)
    level: list[float] = field(default_factory=list)
    active_min: list[float] = field(default_factory=list)
    falling: list[float] = field(default_factory=list)

    def append(self, t: float, sample: HapticsSample) -> None:
        self.times.append(t)
        self.fz_raw.append(float("nan") if sample.fz_raw is None else sample.fz_raw)
        self.fz.append(float("nan") if sample.fz is None else sample.fz)
        self.level.append(level_for_plot(sample.level))
        self.active_min.append(sample.active_min)
        self.falling.append(1.0 if sample.direction is FzDirection.FALLING else 0.0)


class FzLowPass:
    """First-order low-pass (EMA) on Fz."""

    def __init__(self, tau_s: float) -> None:
        self.tau_s = tau_s
        self._y: float | None = None
        self._t: float | None = None

    def reset(self) -> None:
        self._y = None
        self._t = None

    def update(self, x: float | None, t: float) -> float | None:
        if x is None:
            self.reset()
            return None
        if self.tau_s <= 0 or self._y is None or self._t is None:
            self._y = x
            self._t = t
            return x
        dt = t - self._t
        if dt <= 0:
            return self._y
        alpha = dt / (self.tau_s + dt)
        self._y = self._y + alpha * (x - self._y)
        self._t = t
        return self._y


class HystState:
    """Rising/falling from average dFz/dt over a sliding window."""

    def __init__(
        self,
        direction_eps: float = DIRECTION_EPS_DEFAULT,
        direction_window: int = DIRECTION_WINDOW_DEFAULT,
    ) -> None:
        self.direction_eps = direction_eps
        self.direction_window = direction_window
        self.direction = FzDirection.RISING
        self._fz: deque[float] = deque(maxlen=direction_window)
        self._times: deque[float] = deque(maxlen=direction_window)

    def reset(self) -> None:
        self._fz.clear()
        self._times.clear()
        self.direction = FzDirection.RISING

    def on_missing_frame(self) -> None:
        self._fz.clear()
        self._times.clear()

    def update(self, fz: float, t: float) -> FzDirection:
        self._fz.append(fz)
        self._times.append(t)
        if len(self._fz) < 2:
            return self.direction

        dt = self._times[-1] - self._times[0]
        if dt <= 0:
            return self.direction

        rate = (self._fz[-1] - self._fz[0]) / dt
        if rate > self.direction_eps:
            self.direction = FzDirection.RISING
        elif rate < -self.direction_eps:
            self.direction = FzDirection.FALLING
        return self.direction

    def active_min(self, rising_min: float, falling_min: float) -> float:
        if self.direction is FzDirection.RISING:
            return rising_min
        return falling_min


def fz_from_tactile_frame(frame: dict | None) -> float | None:
    """Extract |Fz| (N) from a ``get_latest_tactile`` frame dict."""
    if not frame or "content" not in frame:
        return None
    f6 = frame["content"].get("F6")
    if f6 is None:
        return None
    return abs(float(f6[2]))


def read_fz(hand, finger: str) -> float | None:
    """Read |Fz| for ``finger`` from a connected ``SharpaHand``.

    The host must have called ``hand.enable_tactile({finger: channel})`` first.
    """
    return fz_from_tactile_frame(hand.get_latest_tactile(finger))


def fz_to_level(fz: float, floor: float, signal_max: float) -> int | None:
    if signal_max <= floor:
        raise ValueError("signal_max must be greater than floor")
    if fz < floor:
        return None
    t = (fz - floor) / (signal_max - floor)
    t = max(0.0, min(1.0, t))
    return int(round(t * 9.0))


def map_fz_hyst(
    fz: float | None,
    state: HystState,
    *,
    rising_min: float,
    falling_min: float,
    signal_max: float,
    t: float,
) -> tuple[int | None, float, FzDirection]:
    if fz is None:
        state.on_missing_frame()
        return None, rising_min, state.direction

    direction = state.update(fz, t)
    floor = state.active_min(rising_min, falling_min)
    level = fz_to_level(fz, floor, signal_max)
    return level, floor, direction


def level_for_plot(level: int | None) -> float:
    if level is None:
        return 0.0
    return float(level + 1)


class MotorHaptics:
    """Maps tactile Fz to vibration level on a single motor."""

    def __init__(
        self,
        config: MotorHapticsConfig,
        motor: VibDriver,
        read_fz: Callable[[], float | None],
    ) -> None:
        self.config = config
        self.motor = motor
        self.read_fz = read_fz
        self.hyst = HystState(config.direction_eps, config.direction_window)
        self._fz_lpf = FzLowPass(config.fz_tau_s)
        self._rising_min = config.rising_min
        self._falling_min = config.falling_min
        self._signal_max = config.signal_max
        self._haptics_enabled = True
        self._threshold_test = False

    @classmethod
    def from_sharpa_hand(cls, config: MotorHapticsConfig, motor: VibDriver, hand) -> MotorHaptics:
        return cls(config, motor, lambda: read_fz(hand, config.finger))

    @property
    def rising_min(self) -> float:
        return self._rising_min

    @rising_min.setter
    def rising_min(self, value: float) -> None:
        self._rising_min = value

    @property
    def falling_min(self) -> float:
        return self._falling_min

    @falling_min.setter
    def falling_min(self, value: float) -> None:
        self._falling_min = value

    @property
    def signal_max(self) -> float:
        return self._signal_max

    @signal_max.setter
    def signal_max(self, value: float) -> None:
        self._signal_max = value

    def set_tuning_mode(
        self,
        *,
        haptics_enabled: bool = True,
        threshold_test: bool = False,
        falling_min: float | None = None,
    ) -> None:
        self._haptics_enabled = haptics_enabled
        self._threshold_test = threshold_test
        if falling_min is not None:
            self._falling_min = falling_min

    def reset_hyst(self) -> None:
        self.hyst.reset()
        self._fz_lpf.reset()

    def filter_fz(self, fz_raw: float | None, t: float) -> float | None:
        return self._fz_lpf.update(fz_raw, t)

    def effective_falling_min(self, use_config_falling: bool) -> float:
        if use_config_falling:
            return self._falling_min
        return self._rising_min

    def compute_sample(
        self,
        fz_raw: float | None,
        fz: float | None,
        *,
        t: float,
        use_config_falling: bool = True,
    ) -> HapticsSample:
        if not self._haptics_enabled:
            self.hyst.on_missing_frame()
            return HapticsSample(fz_raw, fz, None, self._rising_min, self.hyst.direction)

        if self._threshold_test:
            level = 9 if (fz is not None and fz >= self._rising_min) else None
            return HapticsSample(fz_raw, fz, level, self._rising_min, FzDirection.RISING)

        falling = self.effective_falling_min(use_config_falling)
        level, active_min, direction = map_fz_hyst(
            fz,
            self.hyst,
            rising_min=self._rising_min,
            falling_min=falling,
            signal_max=self._signal_max,
            t=t,
        )
        return HapticsSample(fz_raw, fz, level, active_min, direction)

    def update(self, *, use_config_falling: bool = True) -> HapticsSample:
        t = time.perf_counter()
        fz_raw = self.read_fz()
        fz = self.filter_fz(fz_raw, t)
        sample = self.compute_sample(
            fz_raw,
            fz,
            t=t,
            use_config_falling=use_config_falling,
        )
        if self._haptics_enabled:
            self.motor.set_motor_level(self.config.motor, sample.level)
        else:
            self.stop()
        return sample

    def stop(self) -> None:
        self.motor.set_motor_level(self.config.motor, None)

    def apply_config(self, config: MotorHapticsConfig) -> None:
        config.validate()
        self.config = config
        self._rising_min = config.rising_min
        self._falling_min = config.falling_min
        self._signal_max = config.signal_max
        self.hyst.direction_eps = config.direction_eps
        self.hyst.direction_window = config.direction_window
        self.hyst._fz = deque(maxlen=config.direction_window)
        self.hyst._times = deque(maxlen=config.direction_window)
        self._fz_lpf = FzLowPass(config.fz_tau_s)


class DittoHaptics:
    """Runs all motors in a ``HapticsConfig``."""

    def __init__(
        self,
        config: HapticsConfig,
        vib_driver: VibDriver,
        read_fz_by_finger: dict[str, Callable[[], float | None]],
    ) -> None:
        self.config = config
        self.vib_driver = vib_driver
        self._motors: dict[int, MotorHaptics] = {}
        for motor_id, motor_cfg in config.motors.items():
            read_fz = read_fz_by_finger[motor_cfg.finger]
            self._motors[motor_id] = MotorHaptics(motor_cfg, vib_driver, read_fz)

    @classmethod
    def from_sharpa_hand(cls, config: HapticsConfig, vib_driver: VibDriver, hand) -> DittoHaptics:
        fingers = {motor_cfg.finger for motor_cfg in config.motors.values()}
        read_fz_by_finger = {finger: (lambda f=finger: read_fz(hand, f)) for finger in fingers}
        return cls(config, vib_driver, read_fz_by_finger)

    def motor(self, motor_id: int) -> MotorHaptics:
        return self._motors[motor_id]

    def update(self, *, use_config_falling: bool = True) -> dict[int, HapticsSample]:
        return {
            motor_id: channel.update(use_config_falling=use_config_falling)
            for motor_id, channel in self._motors.items()
        }

    def stop(self) -> None:
        for channel in self._motors.values():
            channel.stop()
