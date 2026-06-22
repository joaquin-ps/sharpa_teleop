# 🤖 Sharpa Teleop

Dynamixel leader (current mode) → Sharpa Wave follower (position mode) with haptic force feedback from Sharpa joint torques.

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

## 🖐️ Run 3-DoF index teleop

Index MCP AA (motor 21) + MCP flex (22) + PIP (23). From the **repository root**:

```bash
conda activate sharpa_ditto
dynamixel-port --latency-timer 1

python sharpa_teleop/sharpa_teleop_controller.py hand_config=sharpa_3dof_index
```

Or from the `sharpa_teleop/` package directory:

```bash
cd sharpa_teleop
python sharpa_teleop_controller.py hand_config=sharpa_3dof_index
```

**Live diagnostics** (separate terminal):

```bash
python sharpa_teleop/live_plot.py hand_config=sharpa_3dof_index --current --position --torque
```

Override the U2D2 port if needed: `u2d2.usb_port=/dev/ttyUSB0`

## 📚 More documentation

- **[Staged testing & calibration](sharpa_teleop/TELEOP_TESTING.md)** — 1-DoF bring-up, force-feedback stages, live plot, tuning tables
- **`sharpa_teleop/conf/hand_config/`** — per-setup Hydra configs (`sharpa_1dof_*`, `sharpa_2dof_*`, `sharpa_3dof_index`)
- **`sharpa_controller/tools/`** — read Sharpa joints, torques, tactile

## 🏗️ Repository layout

```
sharpa_teleop/              # this repo
├── finger_aloha/           # submodule (branch: sharpa)
├── sharpa_controller/      # submodule
├── sharpa_teleop/          # Python package + configs
│   ├── conf/
│   ├── sharpa_teleop_controller.py
│   ├── live_plot.py
│   └── TELEOP_TESTING.md
└── environment.yml
```
