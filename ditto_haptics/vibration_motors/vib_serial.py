"""Serial helpers for the multi-motor Arduino vib firmware.

Protocol (2 bytes per motor update):
  ``<motor><level>``  e.g. ``05`` = motor 0 at level 5
  ``<motor>x``          e.g. ``1x`` = motor 1 off

Use :func:`broadcast_level` or :class:`VibMotor` to drive motors.
"""

from __future__ import annotations

import time

import serial

# Must match active motor count in vibration_motors/arduino/vib_firmware/vib_firmware.ino
NUM_MOTORS = 2


def level_to_cmd(motor: int, level: int | None) -> bytes:
    if level is None:
        return f"{motor}x".encode("ascii")
    return f"{motor}{level}".encode("ascii")


def broadcast_level(
    ser: serial.Serial,
    level: int | None,
    *,
    num_motors: int = NUM_MOTORS,
) -> None:
    """Send the same level (or stop) to every motor."""
    for motor in range(num_motors):
        ser.write(level_to_cmd(motor, level))


class VibMotor:
    """Thin serial wrapper for per-motor or broadcast vibration control."""

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        *,
        num_motors: int = NUM_MOTORS,
    ) -> None:
        self._serial = serial.Serial(port, baud, timeout=1)
        time.sleep(2.0)  # Arduino resets when the serial port opens.
        self._num_motors = num_motors
        self._last_levels: list[int | None] = [object()] * num_motors  # type: ignore[list-item]

    def set_motor_level(self, motor: int, level: int | None) -> None:
        if motor < 0 or motor >= self._num_motors:
            raise ValueError(f"motor index {motor} out of range 0..{self._num_motors - 1}")
        if level == self._last_levels[motor]:
            return
        self._serial.write(level_to_cmd(motor, level))
        self._last_levels[motor] = level

    def set_levels(self, levels: dict[int, int | None]) -> None:
        """Update multiple motors; skips serial writes when a level is unchanged."""
        for motor, level in levels.items():
            self.set_motor_level(motor, level)

    def set_level(self, level: int | None) -> None:
        """Broadcast the same level to every motor."""
        for motor in range(self._num_motors):
            self.set_motor_level(motor, level)

    def stop(self) -> None:
        self.set_level(None)

    def close(self) -> None:
        self.stop()
        self._serial.close()
