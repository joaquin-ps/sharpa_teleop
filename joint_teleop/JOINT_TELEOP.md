# Joint teleop

Direct **joint-to-joint** mapping: Dynamixel leader (current mode) → Sharpa Wave (position mode), with optional haptic force feedback from Sharpa joint torques.

Install and env setup: see the [repo README](../README.md).

## Run teleop

From the repo root (`sharpa_teleop/`):

```bash
conda activate sharpa_ditto
dynamixel-port --latency-timer 1   # before real hardware

# 3-DoF index (motors 21, 22, 23)
python joint_teleop/sharpa_teleop_controller.py hand_config=sharpa_3dof_index

# Override U2D2 port or use fake Dynamixel (no USB)
python joint_teleop/sharpa_teleop_controller.py hand_config=sharpa_3dof_index u2d2.usb_port=/dev/ttyUSB0
python joint_teleop/sharpa_teleop_controller.py hand_config=sharpa_3dof_index u2d2.fake_u2d2=true
```

From `joint_teleop/`:

```bash
python sharpa_teleop_controller.py hand_config=sharpa_3dof_index
```

## Live plot (diagnostics)

Run in a separate terminal while teleop is active:

```bash
python joint_teleop/live_plot.py hand_config=sharpa_3dof_index --current --position --torque
```

Flags: `--current`, `--position`, `--velocity`, `--torque` (default: current + velocity + position).

## Hand configs

Hydra configs live in `conf/hand_config/`. Select with `hand_config=<name>`:

| Config | Leader motors | Sharpa joints |
|--------|---------------|---------------|
| `sharpa_1dof_index_pip` | 23 | Index PIP |
| `sharpa_1dof_index_mcp` | 22 | Index MCP flex |
| `sharpa_1dof_index_mcp_aa` | 21 | Index MCP AA |
| `sharpa_2dof_index_mcp_pip` | 22, 23 | MCP flex + PIP |
| `sharpa_3dof_index` | 21, 22, 23 | MCP AA + MCP flex + PIP |

Default config (`conf/config.yaml`) starts at `sharpa_1dof_index_pip` for staged bring-up.

Root config also sets `control_frequency`, `verbose`, and `show_current_breakdown` (prints per-joint force-rendering breakdown in the teleop loop).

## Config structure

Each `hand_config` YAML has three main sections:

**`leader`** — Dynamixel motors (IDs, limits, current-mode force rendering per joint)

**`sharpa_mapping.pairs`** — one entry per leader→Sharpa joint pair:

| Field | Purpose |
|-------|---------|
| `leader_motor_id` | Dynamixel ID |
| `sharpa_joint` | Sharpa joint name (must match SDK) |
| `offset_rad` | Add to leader angle before sending to Sharpa |
| `scale` | Multiply leader angle (`sharpa_cmd = scale × leader + offset`) |
| `torque_to_mA` | Sharpa torque (Nm) → synthetic mA for force rendering |
| `torque_filter_alpha` | EMA on torque before scaling (1.0 = no filter) |

**`conf/sharpa/default.yaml`** — which Sharpa joints are enabled and global limits:

| Field | Purpose |
|-------|---------|
| `enabled_joints` | Only these joints receive position commands |
| `speed_coeff` | Sharpa speed limit scale (start low) |
| `current_coeff` | Sharpa current limit scale (start low) |

## Tunable parameters (force feedback)

Per leader joint, under `leader.joint_settings.*.current_control`:

| Field | Purpose |
|-------|---------|
| `enable_force_rendering` | Turn haptics on/off |
| `force_rendering_gain` | Torque → leader current strength |
| `force_rendering_alpha` | EMA on rendered current |
| `force_rendering_threshold_positive` / `_negative` | Deadband (synthetic mA) before force kicks in |
| `force_rendering_max_current` | Cap on rendered current (mA) |
| `enable_force_rendering_damping` | Extra leader damping during active rendering |
| `force_rendering_damping_gain` | Damping strength (mA per rad/s above threshold) |
| `force_rendering_damping_velocity_threshold` | Velocity deadband for damping |
| `force_rendering_damping_max_current` | Cap on damping current (mA) |

Override at runtime without editing YAML:

```bash
python joint_teleop/sharpa_teleop_controller.py \
  hand_config=sharpa_3dof_index \
  hand_config.leader.joint_settings.0.current_control.force_rendering_gain=0.15
```

## Staged bring-up (1-DoF)

1. **Sharpa read-only** — `python sharpa_controller/tools/read_joints.py --once --radians`
2. **Open loop** — `enable_force_rendering=false` on the active joint
3. **Force path wired, gain 0** — `enable_force_rendering=true force_rendering_gain=0.0`
4. **Closed loop** — raise `force_rendering_gain` and tune `torque_to_mA` gradually

Calibrate `zero_position` (leader encoder at mechanical zero) and `offset_rad` / `scale` so leader and Sharpa report the same pose at a known configuration.

## Safety

- Start with low `sharpa.speed_coeff` and `sharpa.current_coeff`
- Only enable the Sharpa joints listed in `enabled_joints`
- Ctrl+C stops teleop; leader torque is disabled on disconnect
