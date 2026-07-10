# Ditto Haptics

Tactile force (Fz) from a Sharpa hand drives vibration motors over serial. Each motor has its own config entry (finger, tactile channel, thresholds).

## Table of Contents

- 📁 [Layout](#layout)
- 🛠️ [Setup](#setup)
  - 🐍 [Conda environment](#1-conda-environment)
  - ⚡ [Arduino firmware](#2-arduino-firmware)
  - ⌨️ [Test motors (no hand required)](#3-test-motors-no-hand-required)
- ⚙️ [Config](#config)
- ▶️ [Usage](#usage)
- 🔌 [Embedding in teleop](#embedding-in-teleop)

## Layout

```
sharpa_teleop/ditto_haptics/
  ditto_haptics.py          # library (embed in teleop)
  run_ditto_haptics.py      # dev runner, plots, tuning
  config/                   # YAML motor configs
  vibration_motors/
    vib_serial.py           # serial protocol
    keyboard_test.py        # manual motor test
    arduino/vib_firmware/   # firmware to flash
```

## Setup

### 1. Conda environment

```bash
cd sharpa_teleop/ditto_haptics
conda env create -f environment.yaml
conda activate ditto_haptx
```

### 2. Arduino firmware

1. Open `vibration_motors/arduino/vib_firmware/vib_firmware.ino` in the Arduino IDE.
2. Set `NUM_MOTORS` and `motorPins[]` to match your hardware.
3. Flash the board and note the serial port (e.g. `/dev/ttyACM0`).

Update `NUM_MOTORS` in `vibration_motors/vib_serial.py` to match the firmware.

### 3. Test motors (no hand required)

```bash
cd sharpa_teleop/ditto_haptics
python -m vibration_motors.keyboard_test
```

Hold `0`–`9` or space for max; ESC to quit. All motors receive the same level.

## Config

Configs are motor-keyed YAML files in `config/`. Example (`config/thumb_index.yaml`):

```yaml
plot_motor: 0   # live plot shows this motor only; omit to disable
motors:
  0:
    finger: thumb
    tactile_channel: 4
    rising_min: 0.05
    falling_min: 0.3
    signal_max: 3.0
    direction_window: 5
    direction_eps: 0.05
    fz_tau_s: 0.02
  1:
    finger: thumb
    tactile_channel: 4
    rising_min: 0.05
    falling_min: 0.3
    signal_max: 3.0
```

| Field | Meaning |
|-------|---------|
| `finger` | Sharpa finger name |
| `tactile_channel` | SDK channel (0=pinky … 4=thumb) |
| `rising_min` | Fz floor while force is rising (N) |
| `falling_min` | Fz floor while force is falling (N) |
| `signal_max` | Fz at vibration level 9 (N) |
| `direction_window` | Samples for average dFz/dt |
| `direction_eps` | Min \|dFz/dt\| (N/s) to switch rising/falling |
| `fz_tau_s` | Fz low-pass time constant (s); 0 = off |

Top-level `plot_motor` selects which motor id to show in the live plot. Omit it (or leave unset) to run haptics without a live plot window.

Two motors on the same tactile signal: add two motor ids with the same `finger` / `tactile_channel` (settings can match or differ).

## Usage

From `sharpa_teleop/ditto_haptics`:

```bash
# Live haptics + plot
python run_ditto_haptics.py config/thumb.yaml

# Record a trace
python run_ditto_haptics.py config/thumb.yaml --plot --duration 8

# Interactive tuning — all motors in config (use --motor N for one)
python run_ditto_haptics.py config/thumb_index.yaml --tune
```

Common flags: `--vib-port`, `--sharpa-serial`, `--motor N`, `--no-live-plot`.

## Embedding in teleop

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path("sharpa_teleop/ditto_haptics")))
sys.path.insert(0, str(Path("sharpa_teleop/sharpa_controller")))

from ditto_haptics import DittoHaptics, HapticsConfig
from vibration_motors.vib_serial import VibMotor

cfg = HapticsConfig.load("sharpa_teleop/ditto_haptics/config/thumb.yaml")
hand.enable_tactile(cfg.tactile_enable_map())
vib = VibMotor("/dev/ttyACM0")
haptics = DittoHaptics.from_sharpa_hand(cfg, vib, hand)

while running:
    haptics.update()
```
