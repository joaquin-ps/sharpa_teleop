# 🤖 DITTO Sharpa

Ditto leader → Sharpa Wave teleop (joint map and/or pad retargeting) plus
fingertip vibration haptics:

| Package | What it does |
|---------|----------------|
| **`sharpa_teleop/`** | Ditto → Sharpa teleop: retarget IK and/or joint mapping + force rendering (2f / 3f) |
| **`ditto_haptics/`** | Tactile Fz → vibration motors (standalone; embeddable in teleop loops) |

## 📑 Table of contents

- [📦 Installation](#-installation)
- [▶️ Usage](#️-usage)
  - [🎯 Teleop](#-teleop)
  - [📳 Haptics](#-haptics)
- [📚 Documentation](#-documentation)
- [🏗️ Layout](#️-layout)

## 📦 Installation

### 1. Clone with submodules

```bash
git clone --recurse-submodules git@github.com:joaquin-ps/ditto_sharpa.git
cd ditto_sharpa
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Conda environment

```bash
conda env create -f environment.yml
conda activate ditto_sharpa
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

From the **repository root**:

```bash
conda activate ditto_sharpa

# Retargeting core (imports + one IK step)
python -c "
import sys; sys.path[:0] = ['sharpa_teleop', '.']
import numpy as np
from retargeting import DittoToSharpaRetargeter
r = DittoToSharpaRetargeter()
print('retarget ok', r.retarget(np.zeros(7), np.zeros(22)).index_residual)
"

# Teleop config loads (retarget + joint)
python -c "
import sys; sys.path.insert(0, 'sharpa_teleop')
from hydra import compose, initialize_config_dir
from _paths import CONF_DIR, DITTO_CONF_DIR
with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
    cfg = compose(config_name='config', overrides=[
        f'hydra.searchpath=[file://{DITTO_CONF_DIR}]',
        'hand_config=joint/sharpa_3dof_index',
    ])
print('teleop config ok', cfg.hand_config.control.fingers.index)
"
```

## ▶️ Usage

### 🎯 Teleop

> **Note:** before any run that talks to the Ditto U2D2, set the USB latency timer
> (requires `sudo` when prompted): `dynamixel-port --latency-timer 1`

**URDF viewer only** (sliders + live retargeting, no hardware):

```bash
python sharpa_teleop/viz/view_assets.py
```

**Physical Ditto leader + Sharpa follower + viewer** (hardware is opt-in):

```bash
python sharpa_teleop/viz/view_teleop.py                   # viewer only
python sharpa_teleop/viz/view_teleop.py --ditto --sharpa  # both hardware
python sharpa_teleop/viz/view_teleop.py --ditto --sharpa --3f
python sharpa_teleop/viz/view_teleop.py --ditto u2d2.fake_u2d2=true
```

**Main force feedback teleop script** (`run_teleop.py`) always connects Ditto + Sharpa. Mode comes
from the top-level `hand_config` (retarget, joint map, or blend):

2-finger / 3-finger — retarget IK + tactile force:

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_2f_tactile
```

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_3f_tactile
```

2-finger / 3-finger — blended sources (index/middle: 1:1 joint + tactile/measured; thumb: IK + tactile/estimate):

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_2f_blend
```

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_3f_blend
```

Direct joint map + measured force:

```bash
python sharpa_teleop/run_teleop.py hand_config=joint/sharpa_3dof_index
```

```bash
python sharpa_teleop/run_teleop.py hand_config=joint/sharpa_3dof_middle
```

**Live diagnostics**:

```bash
python sharpa_teleop/viz/force_plot.py hand_config=ditto_2f_tactile
```

```bash
python sharpa_teleop/viz/force_plot.py hand_config=joint/sharpa_3dof_index
```

Fake Dynamixel (no USB): append `u2d2.fake_u2d2=true`

Override the U2D2 port: `u2d2.usb_port=/dev/ttyUSB0`

See **[sharpa_teleop/TELEOP.md](sharpa_teleop/TELEOP.md)** and
**[docs/COMMANDS.md](sharpa_teleop/docs/COMMANDS.md)** for more.

### 📳 Haptics

`ditto_haptics/` maps Sharpa fingertip tactile **Fz** to serial vibration motors. Standalone library (host owns the Sharpa hand); run alone or embed in a teleop loop.

```bash
python ditto_haptics/run_ditto_haptics.py
```

See **[ditto_haptics/HAPTICS.md](ditto_haptics/HAPTICS.md)** for setup, motor configs, and embedding.

## 📚 Documentation

- **[Ditto–Sharpa teleop](sharpa_teleop/TELEOP.md)** — viewer, retarget/joint modes, force rendering, IK API
- **[Haptics](ditto_haptics/HAPTICS.md)** — vibration motors from tactile Fz
- **`sharpa_teleop/conf/hand_config/`** — top-level Hydra configs (`ditto_2f_*`, `ditto_3f_*`, `joint/*`)
- **`sharpa_controller/tools/`** — read Sharpa joints, torques, tactile

## 🏗️ Layout

```
ditto_sharpa/                       # this repo
├── ditto/                          # submodule (Dynamixel leader)
├── sharpa_controller/              # submodule (Sharpa Wave)
├── sharpa_teleop/                  # Ditto → Sharpa teleop (retarget + joint + force)
│   ├── conf/hand_config/           # ditto_2f_* / ditto_3f_* / joint/*
│   ├── docs/                       # COMMANDS, FORCE_RENDERING, TELEOP_EQS
│   ├── hardware_interfaces/
│   ├── retargeting/                # IK / pad retargeting
│   ├── teleop/                     # DittoSharpaEngine + DittoSharpaTeleop
│   ├── viz/
│   ├── run_teleop.py
│   └── TELEOP.md
├── ditto_haptics/                  # tactile Fz → vibration motors
│   ├── ditto_haptics.py
│   ├── run_ditto_haptics.py
│   └── HAPTICS.md
└── environment.yml
```
