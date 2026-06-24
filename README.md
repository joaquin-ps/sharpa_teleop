# 🤖 Sharpa Teleop

Two teleop stacks sharing `finger_aloha` (Dynamixel leader) and `sharpa_controller` (Sharpa Wave):

| Package | What it does |
|---------|----------------|
| **`joint_teleop/`** | Direct joint-to-joint mapping with force feedback |
| **`retargeting_teleop/`** | Ditto leader → Sharpa kinematic retargeting (index + thumb pads) |

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

## 🔌 Before each session

Set the U2D2 USB latency timer **before** running teleop scripts (requires `sudo` when prompted):

```bash
dynamixel-port --latency-timer 1
```

## 🧪 Quick smoke tests (no hardware)

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
from joint_teleop._paths import CONF_DIR, FA_CONF_DIR
with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
    cfg = compose(config_name='config', overrides=[f'hydra.searchpath=[file://{FA_CONF_DIR}]'])
print('joint teleop config ok', cfg.hand_config.leader.mode)
"
```

## 🖐️ Joint teleop (direct mapping + force feedback)

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

## 🎯 Retargeting teleop (Ditto leader → Sharpa pads)

**URDF viewer only** (sliders + live retargeting, no hardware):

```bash
python retargeting_teleop/viz/view_assets.py
```

**Physical Ditto leader + viewer**:

```bash
python retargeting_teleop/viz/view_teleop.py
python retargeting_teleop/viz/view_teleop.py u2d2.fake_u2d2=true   # no USB
```

Tune defaults in `retargeting_teleop/retargeting/retargeter.py` (`index_cartesian_scale`, `thumb_*_weight`, etc.).

See **[retargeting_teleop/RETARGETING_TELEOP.md](retargeting_teleop/RETARGETING_TELEOP.md)** for viewer usage and code API.

## 📚 More documentation

- **[Joint teleop](joint_teleop/JOINT_TELEOP.md)** — run commands, configs, force-feedback tuning
- **[Retargeting teleop](retargeting_teleop/RETARGETING_TELEOP.md)** — viewer and IK API
- **`joint_teleop/conf/hand_config/`** — per-setup Hydra configs
- **`sharpa_controller/tools/`** — read Sharpa joints, torques, tactile

## 🏗️ Repository layout

```
sharpa_teleop/                      # this repo
├── finger_aloha/                   # submodule (Dynamixel leader)
├── sharpa_controller/              # submodule (Sharpa Wave)
├── joint_teleop/                   # direct joint teleop
│   ├── conf/
│   ├── sharpa_teleop_controller.py
│   ├── live_plot.py
│   └── JOINT_TELEOP.md
├── retargeting_teleop/             # pad retargeting + dev viewer
│   ├── conf/
│   ├── hardware_interfaces/
│   ├── retargeting/
│   ├── viz/
│   └── RETARGETING_TELEOP.md
└── environment.yml
```
