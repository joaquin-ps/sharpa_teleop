"""Manual keyboard test for vibration motors (all motors broadcast).

Run from the ditto_haptics directory::

    python -m vibration_motors.keyboard_test
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import serial
from pynput import keyboard

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from vibration_motors.vib_serial import broadcast_level  # noqa: E402

PORT = "/dev/ttyACM0"  # update to your Arduino's port
BAUD = 115200

arduino = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # wait for Arduino to reset after serial connection opens

active_key = None  # tracks which key is currently driving the motors


def key_to_level(key) -> int | None:
    """Map a pynput key to a vibration level 0-9, or None if irrelevant."""
    if key == keyboard.Key.space:
        return 9  # space = max intensity, same as digit 9
    try:
        if key.char is not None and key.char in "0123456789":
            return int(key.char)
    except AttributeError:
        pass
    return None


def on_press(key):
    global active_key
    level = key_to_level(key)
    if level is not None and active_key is None:
        active_key = key
        broadcast_level(arduino, level)


def on_release(key):
    global active_key
    if key == active_key:
        broadcast_level(arduino, None)
        active_key = None
    if key == keyboard.Key.esc:
        broadcast_level(arduino, None)
        return False  # stops the listener


def main() -> None:
    print("Hold 0-9 for intensity levels, SPACE for max. Press ESC to quit.")
    print(f"Broadcasting to {arduino.name} (all motors).")

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    broadcast_level(arduino, None)
    arduino.close()
    print("Closed connection.")


if __name__ == "__main__":
    main()
