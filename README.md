# 🤖 Sharpa Teleop

Two teleop stacks sharing `ditto` (Dynamixel leader) and `sharpa_controller` (Sharpa Wave), plus fingertip vibration haptics:

| Package | What it does |
|---------|----------------|
| **`retargeting_teleop/`** | Ditto → Sharpa pad retargeting + force rendering (2f / 3f) |
| **`joint_teleop/`** | Direct joint-to-joint mapping with force feedback |
| **`ditto_haptics/`** | Tactile Fz → vibration motors (standalone; embeddable in teleop loops) |

## 📑 Table of contents

- [📦 Installation](#-installation)
- [▶️ Usage](#️-usage)
  - [🎯 Retargeting](#-retargeting)
  - [🖐️ Joint teleop](#️-joint-teleop)
  - [📳 Haptics](#-haptics)
- [📚 Documentation](#-documentation)
- [🏗️ Layout](#️-layout)

## 📦 Installation

### 1. Clone with submodules

```bash
git clone --recurse-submodules git@github.com:joaquin-ps/sharpa_teleop.git
cd sharpa_teleop
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Conda environment

```bash
conda env create -f environment.yml
conda activate sharpa_ditto
```

Requires **Python 3.10, 3.11, or 3.12** (matches the Sharpa Wave SDK Python bindings).

### 3. Sharpa Wave SDK

Install the official SDK on your machine. Downloads and platform-specific steps (`.deb` on x86_64, `.zip` on ARM64) are documented here:

👉 **[sharpa-robotics/sharpa-wave-sdk](https://github.com/sharpa-robotics/sharpa-wave-sdk)**

After install, the SDK should live at `/opt/sharpa-wave-sdk/`. Quick check:

```bash
python -c "import sys; sys.path.insert(0, '/opt/sharpa-wave-sdk/python'); import sharpa; print('OK')"
```

### 4. Smoke tests (no hardware)

From the **repository root** (`sharpa_teleop/`):

```bash
conda activate sharpa_ditto

# Retargeting core (imports + one IK step)
python -c "
import sys; sys.path[:0] = ['retargeting_teleop', '.']
import numpy as np
from retargeting import DittoToSharpaRetargeter
r = DittoToSharpaRetargeter()
print('retarget ok', r.retarget(np.zeros(7), np.zeros(22)).index_residual)
"

# Joint teleop config loads
python -c "
import sys; sys.path.insert(0, '.')
from hydra import compose, initialize_config_dir
from joint_teleop._paths import CONF_DIR, DITTO_CONF_DIR
with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
    cfg = compose(config_name='config', overrides=[f'hydra.searchpath=[file://{DITTO_CONF_DIR}]'])
print('joint teleop config ok', cfg.hand_config.leader.mode)
"
```

## ▶️ Usage

### 🎯 Retargeting

> **Note:** before any run that talks to the Ditto U2D2, set the USB latency timer
> (requires `sudo` when prompted): `dynamixel-port --latency-timer 1`

**URDF viewer only** (sliders + live retargeting, no hardware):

```bash
python retargeting_teleop/viz/view_assets.py
```

**Physical Ditto leader + Sharpa follower + viewer** (hardware is opt-in):

```bash
python retargeting_teleop/viz/view_teleop.py                   # viewer only
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa  # both hardware
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa --3f
python retargeting_teleop/viz/view_teleop.py --ditto u2d2.fake_u2d2=true
```

**Force rendering** (`run_force_render.py`) always connects Ditto + Sharpa. Mode comes from the top-level `hand_config`:

```bash
# 2f / 3f — retarget IK + tactile force
python retargeting_teleop/run_force_render.py hand_config=ditto_2f_tactile
python retargeting_teleop/run_force_render.py hand_config=ditto_3f_tactile

# 2f / 3f — blended sources (index/middle: joint + tactile/measured; thumb: IK + tactile/estimate)
python retargeting_teleop/run_force_render.py hand_config=ditto_2f_blend
python retargeting_teleop/run_force_render.py hand_config=ditto_3f_blend
```

See **[retargeting_teleop/RETARGETING_TELEOP.md](retargeting_teleop/RETARGETING_TELEOP.md)** and **[docs/COMMANDS.md](retargeting_teleop/docs/COMMANDS.md)** for more.

### 🖐️ Joint teleop

> **Note:** before any run that talks to the Ditto U2D2, set the USB latency timer
> (requires `sudo` when prompted): `dynamixel-port --latency-timer 1`

Index MCP AA (motor 21) + MCP flex (22) + PIP (23):

```bash
python joint_teleop/sharpa_teleop_controller.py hand_config=sharpa_3dof_index
```

**Live diagnostics**:

```bash
python joint_teleop/live_plot.py hand_config=sharpa_3dof_index --current --position --torque
```

Fake Dynamixel (no USB): append `u2d2.fake_u2d2=true`

Override the U2D2 port: `u2d2.usb_port=/dev/ttyUSB0`

### 📳 Haptics

`ditto_haptics/` maps Sharpa fingertip tactile **Fz** to serial vibration motors. Standalone library (host owns the Sharpa hand); run alone or embed in a teleop loop.

```bash
python ditto_haptics/run_ditto_haptics.py
```

See **[ditto_haptics/HAPTICS.md](ditto_haptics/HAPTICS.md)** for setup, motor configs, and embedding.

## 📚 Documentation

- **[Retargeting teleop](retargeting_teleop/RETARGETING_TELEOP.md)** — viewer, force-rendering modes/configs, IK API
- **[Joint teleop](joint_teleop/JOINT_TELEOP.md)** — run commands, configs, force-feedback tuning
- **[Haptics](ditto_haptics/HAPTICS.md)** — vibration motors from tactile Fz
- **`retargeting_teleop/conf/hand_config/`** — top-level Hydra configs (`ditto_2f_*`, `ditto_3f_*`)
- **`sharpa_controller/tools/`** — read Sharpa joints, torques, tactile

## 🏗️ Layout

```
sharpa_teleop/                      # this repo
├── ditto/                          # submodule (Dynamixel leader)
├── sharpa_controller/              # submodule (Sharpa Wave)
├── retargeting_teleop/             # pad retargeting + force rendering + viewer
│   ├── conf/hand_config/           # ditto_2f_* / ditto_3f_* (tactile, blend, leader_only)
│   ├── docs/                       # COMMANDS, FORCE_RENDERING, TELEOP_EQS
│   ├── hardware_interfaces/
│   ├── retargeting/
│   ├── teleop/                     # engine + force render + force sources
│   ├── viz/
│   ├── run_force_render.py
│   └── RETARGETING_TELEOP.md
├── joint_teleop/                   # direct joint teleop
│   ├── conf/
│   ├── sharpa_teleop_controller.py
│   ├── live_plot.py
│   └── JOINT_TELEOP.md
├── ditto_haptics/                  # tactile Fz → vibration motors
│   ├── ditto_haptics.py
│   ├── run_ditto_haptics.py
│   └── HAPTICS.md
└── environment.yml
```
