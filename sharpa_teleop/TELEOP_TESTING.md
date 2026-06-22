# 🧪 Sharpa + Dynamixel Teleop Testing

Staged bring-up for the 1-DoF index PIP setup (Dynamixel leader → Sharpa index PIP).

See the [top-level README](../README.md) for installation and the 3-DoF quick start.

## 📋 Prerequisites

- [Sharpa Wave SDK](https://github.com/sharpa-robotics/sharpa-wave-sdk) installed at `/opt/sharpa-wave-sdk`
- U2D2 on `/dev/ttyUSB0` (override with `u2d2.usb_port=...`)
- Motor ID **1**, baud **4000000**, XC330-T181 (uses `XC330` motor model in config)
- Conda env: `conda env create -f environment.yml && conda activate sharpa_ditto`
- Install finger_aloha Dynamixel driver (editable):  
  `pip install -e finger_aloha/dynamixel_u2d2`
- Before teleop: `dynamixel-port --latency-timer 1`

## 1️⃣ Stage 1 — Sharpa read-only

Verify Sharpa connectivity and index PIP state:

```bash
python sharpa_controller/tools/read_joints.py --once --radians
python sharpa_controller/tools/read_torques.py --once
```

## 2️⃣ Stage 2 — Open-loop teleop (no force feedback)

Disable force rendering so the leader moves freely while Sharpa tracks position:

```bash
python sharpa_teleop_controller.py \
  hand_config.leader.joint_settings.0.current_control.enable_force_rendering=false
```

**Verify:** Moving the leader flexes only Sharpa index PIP; other Sharpa joints stay fixed.

## 📈 Live plot (diagnostics)

Visualize leader vs Sharpa signals while teleop runs (close the window to stop):

```bash
python live_plot.py
python live_plot.py --current --position --torque
python live_plot.py show_current_breakdown=true
```

Plots:
- **Current:** leader measured (blue), leader commanded (cyan dashed), synth mA raw (green), synth mA filtered (orange, input to force rendering), force rendering damping (purple dotted)
- **Position / velocity:** leader Dynamixel vs Sharpa joint
- **Torque:** Sharpa Nm raw (magenta) vs filtered (orange, `--torque`)

Red dashed lines on the current plot mark `force_rendering_threshold_positive` / `force_rendering_threshold_negative` (in synthetic mA units).

## 3️⃣ Stage 3 — Force feedback plumbing (gain = 0)

Confirm the feedback path is wired without haptic effect:

```bash
python sharpa_teleop_controller.py \
  hand_config.leader.joint_settings.0.current_control.enable_force_rendering=true \
  hand_config.leader.joint_settings.0.current_control.force_rendering_gain=0.0
```

## 4️⃣ Stage 4 — Conservative closed-loop force feedback

Default config uses very low gain (`0.02`), high threshold (`100`), and `max_current: 300`:

```bash
python sharpa_teleop_controller.py
```

Increase gain gradually only after open-loop mapping is calibrated:

```bash
python sharpa_teleop_controller.py \
  hand_config.leader.joint_settings.0.current_control.force_rendering_gain=0.03 \
  hand_config.sharpa_mapping.pairs.0.torque_to_mA=60.0
```

## 🎯 Calibration

Edit [`conf/hand_config/sharpa_1dof_index_pip.yaml`](conf/hand_config/sharpa_1dof_index_pip.yaml):

| Field | Purpose |
|-------|---------|
| `leader.joint_settings.0.joint_settings.zero_position` | Encoder count at mechanical zero |
| `sharpa_mapping.pairs.0.offset_rad` | Align leader and Sharpa joint zeros |
| `sharpa_mapping.pairs.0.scale` | Map leader travel to Sharpa 0–20° range |
| `sharpa_mapping.pairs.0.torque_to_mA` | Nm → synthetic mA for force rendering |
| `sharpa_mapping.pairs.0.torque_filter_alpha` | Input EMA on Sharpa torque before × torque_to_mA (1.0 = off) |
| `current_control.enable_force_rendering_damping` | Extra leader damping during active force rendering |
| `current_control.force_rendering_damping_gain` | Damping gain (mA per rad/s above velocity threshold) |
| `current_control.force_rendering_damping_velocity_threshold` | Velocity deadband (rad/s); damping ramps as `-gain × max(0, \|vel\| − threshold)` |
| `current_control.force_rendering_damping_max_current` | Cap on \|damping\| (mA); default 200 |
| `current_control.force_rendering_max_current` | Cap on \|force rendering\| (mA); default 200 |
| `force_rendering_threshold_positive` / `force_rendering_threshold_negative` | Per-sign synth mA deadbands for force rendering and damping gate; fall back to `force_rendering_threshold` |

1. Place hand at a known pose; read Sharpa index PIP with `read_joints.py --radians`
2. Read leader joint angle at the same pose (or note encoder count)
3. Adjust `zero_position` and `offset_rad` until both report the same angle
4. Flip `scale` sign or `torque_to_mA` sign if feedback pushes the wrong way

## ⚠️ Safety

- Start with low `sharpa.speed_coeff` and `sharpa.current_coeff` (defaults: 0.2 / 0.4)
- Only `"Index PIP Flexion/Extension"` is enabled on Sharpa by default
- Use Ctrl+C to stop; leader torque is disabled on disconnect
