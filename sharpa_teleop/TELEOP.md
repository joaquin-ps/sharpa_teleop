# Ditto–Sharpa teleop

Maps a Ditto Dynamixel leader to a Sharpa Wave follower with optional **force
rendering** (haptics) back to the leader. Per finger you choose how **position**
is driven (IK retarget or direct joint map) and where the rendered **force**
comes from (estimate, tactile, measured Sharpa joint torque, or a blend).

Install + env: see the [repo README](../README.md). Run everything from the
repo root with `conda activate ditto_sharpa`. Before any real Ditto hardware:
`dynamixel-port --latency-timer 1`.

See also: [`docs/COMMANDS.md`](docs/COMMANDS.md) (copy-paste demo commands),
[`docs/FORCE_RENDERING.md`](docs/FORCE_RENDERING.md) and
[`docs/TELEOP_EQS.md`](docs/TELEOP_EQS.md) (the math + mode options).

## Viewer (no hardware)

URDFs, joint sliders, live Ditto → Sharpa retargeting:

```bash
python sharpa_teleop/viz/view_assets.py                 # both hands
python sharpa_teleop/viz/view_assets.py --leader-only   # Ditto only
python sharpa_teleop/viz/view_assets.py --sharpa-only   # Sharpa only
python sharpa_teleop/viz/view_assets.py --3f            # 3-finger Ditto URDF
```

## Teleop with viewer (hardware opt-in)

Hardware is **off by default** (Viser only). Pass `--ditto` and/or `--sharpa`
explicitly to connect hardware.

```bash
python sharpa_teleop/viz/view_teleop.py                       # viewer only
python sharpa_teleop/viz/view_teleop.py --ditto --sharpa      # both hardware
python sharpa_teleop/viz/view_teleop.py --ditto --sharpa hand_config=ditto_2f_tactile
python sharpa_teleop/viz/view_teleop.py --ditto u2d2.fake_u2d2=true
```

| Flag | Effect |
| --- | --- |
| `--ditto` | Enable Ditto leader hardware |
| `--sharpa` | Enable Sharpa follower hardware |
| `--leader-only` / `--sharpa-only` | Show a single hand URDF |
| `--3f` | Use the 3-finger Ditto leader URDF (index + middle + thumb) |

The control mode is **config-only** (`hand_config.control.fingers`). The viewer
has a live **Force source** dropdown (`estimate` | `tactile`) for interactive
comparison of pad-force arrows.

The teleop core lives in `teleop/engine.py` (`DittoSharpaEngine`:
`poll_leader()`, `retarget()`, `estimate_force_feedback()`). The viewer and
headless runner (`run_teleop.py` / `DittoSharpaTeleop`) drive the same engine.

## Headless teleop (`run_teleop.py`)

Always connects Ditto + Sharpa. Mode comes entirely from `hand_config`
(retarget IK, joint map, or a mix). A per-finger summary is printed at startup.

```bash
# Retarget + tactile force
python sharpa_teleop/run_teleop.py hand_config=ditto_2f_tactile
python sharpa_teleop/run_teleop.py hand_config=ditto_3f_tactile

# Blend (index/middle joint+blend, thumb retarget+blend)
python sharpa_teleop/run_teleop.py hand_config=ditto_2f_blend
python sharpa_teleop/run_teleop.py hand_config=ditto_3f_blend

# Direct joint map + measured force
python sharpa_teleop/run_teleop.py hand_config=joint/sharpa_3dof_index
python sharpa_teleop/run_teleop.py hand_config=joint/sharpa_3dof_middle

# Raise a per-joint gain at runtime (index PIP = joint_settings index 2):
python sharpa_teleop/run_teleop.py hand_config=ditto_2f_tactile \
  hand_config.leader.joint_settings.2.current_control.force_rendering_gain=0.05
```

Fake Dynamixel (no USB): append `u2d2.fake_u2d2=true`. Override the U2D2 port:
`u2d2.usb_port=/dev/ttyUSB0`.

### Configs (`conf/hand_config/`)

Top-level configs (preferred). Joint-only bring-up configs live under `joint/`.
Per-finger / legacy YAMLs live under `fingers/` and `old/`.

| Config | Fingers | Position | Force |
| --- | --- | --- | --- |
| `ditto_2f_leader_only` | index+thumb | — | none (viewer/teleop default) |
| `ditto_3f_leader_only` | index+middle+thumb | — | none (3f viewer/teleop default) |
| `ditto_2f_tactile` | index+thumb | retarget | tactile |
| `ditto_3f_tactile` | index+middle+thumb | retarget | tactile |
| `ditto_2f_blend` | index+thumb | index=joint, thumb=retarget | index=50/50 tactile+measured, thumb=65/35 tactile+estimate |
| `ditto_3f_blend` | index+middle+thumb | index/middle=joint, thumb=retarget | index/middle=50/50 tactile+measured, thumb=65/35 tactile+estimate |
| `joint/sharpa_1dof_index_pip` | index PIP | joint | measured |
| `joint/sharpa_1dof_index_mcp` | index MCP flex | joint | measured |
| `joint/sharpa_1dof_index_mcp_aa` | index MCP AA | joint | measured |
| `joint/sharpa_2dof_index_mcp_pip` | index MCP+PIP | joint | measured |
| `joint/sharpa_3dof_index` | index 3-DoF | joint | measured |
| `joint/sharpa_3dof_middle` | middle 3-DoF | joint | measured |

Per-config tuning (gains, `torque_to_mA`, thresholds, EMA `force_rendering_alpha`
/ `torque_filter_alpha`, damping) lives under `leader.joint_settings.*` and
`force_render.joints`. Estimate and tactile are **independent tuning surfaces**
(sensor scale ≠ model estimate).

### Control modes (per finger)

Each finger declares **two independent choices** in
`hand_config.control.fingers`: how its **position** is driven and where its
**force feedback** comes from. This is the single source of truth — there are no
force-source CLI flags, and the startup banner echoes the resolved mode.

**Position** — how the Sharpa finger is driven:

| `position` | Meaning |
| --- | --- |
| `retarget` | Cartesian IK: match the Ditto fingerpad pose |
| `joint` | direct leader→Sharpa joint map (1:1); needs `control.joint_map` |

**Force** — where the haptic feedback rendered to the leader comes from:

| `force` | Meaning |
| --- | --- |
| `estimate` | task-space: model-based `Jᵀ` solve of contact force from Sharpa joint torques |
| `tactile` | task-space: fingertip tactile F6 sensor, rotated into the Sharpa base frame |
| `mix` | config shortcut: 50/50 blend of `estimate` + `tactile` (or use a weight dict) |
| `measured` | joint-space: raw measured Sharpa joint torque; needs `control.joint_map` |

The `estimate`/`tactile` sources (and weighted blends / `mix`) all output force
in the same Sharpa-base frame at the pad, so they are interchangeable per finger
(you can run `index: estimate`, `thumb: tactile`). The viewer dropdown only
toggles `estimate` vs `tactile` for live arrow comparison. `measured` skips the
model and feeds the Sharpa joint torque straight through.

Two shortcut strings exist: `retarget` == `{position: retarget, force: estimate}`
and `joint` == `{position: joint, force: measured}`.

```yaml
control:
  fingers:
    index: {position: joint,    force: measured}   # 1:1 joint map + joint-torque force
    thumb: {position: retarget, force: tactile}    # IK position + tactile force
  # Required for any finger using position=joint and/or force=measured. `scale`
  # sets the sign for BOTH position (θ_sharpa = scale·θ_ditto) and force
  # (τ_ditto = scale·τ_sharpa).
  joint_map:
    index:
    - {ditto_joint: index_joint_0, sharpa_joint: right_index_MCP_AA, scale: 1.0}
    - {ditto_joint: index_joint_1, sharpa_joint: right_index_MCP_FE, scale: 1.0}
    - {ditto_joint: index_joint_2, sharpa_joint: right_index_PIP,    scale: 1.0}
```

The Sharpa follower is always connected for headless teleop. Two more config-only
options live under `force_render`: `calibrate: true` (calibrate tactile on
startup) and `debug: true` (print raw F6 vs base force). Per-motor numeric tuning
(`torque_to_mA`, `torque_filter_alpha`) stays under `force_render.joints`, and
the rendering gains/thresholds/damping under `leader.joint_settings.*` — these
are tuning, not mode selection. Tactile sensor sign + sensor→link rotation are
calibration hooks in `hardware_interfaces/sharpa_follower/conventions.py`
(`SHARPA_TACTILE_FORCE_SIGN`, `SHARPA_TACTILE_SENSOR_LINK_RIGHT`,
`SHARPA_TACTILE_SENSOR_TO_LINK_RPY`).

### Live plot

```bash
python sharpa_teleop/viz/force_plot.py hand_config=ditto_2f_tactile
python sharpa_teleop/viz/force_plot.py hand_config=joint/sharpa_3dof_index
```

Left column: estimated joint torque (Nm), raw vs filtered. Right column: force
rendering vs damping vs net rendered current vs measured leader current (mA),
deadband in red. Joints with `plot_joint: false` in the config are skipped.

## Configs

- Ditto: `conf/hand_config/*.yaml` (default `ditto_2f_leader_only`). Override
  with `hand_config=...`. `motor_models` / `joint_configs` come from
  `ditto` via the Hydra searchpath.
- Sharpa: `conf/sharpa/default.yaml` (serial, enabled joints, speed/current
  coeffs, IO rate). `angle_overrides_deg` widens the SDK position-clamp ROM per
  joint without editing the `sharpa_controller` submodule.

Hardware adapters in `hardware_interfaces/`: `ditto_leader/` (encoder reads),
`sharpa_follower/` (sends follower `sharpa_q`; `conventions.py` maps URDF →
SDK joints + calibration hooks, `session.py` owns the SDK connection).

## Viewer quick reference

- **Green** — `retarget_base` on each hand
- **Cyan / orange (on mesh)** — achieved Ditto / Sharpa pad frames
- **Pale cyan / orange (floating)** — Sharpa IK targets
- GUI folders — cartesian scale and position/orientation IK weights per finger

Tune retarget defaults in `retargeting/retargeter.py` (`index_cartesian_scale`,
`thumb_*_weight`, etc.).

## Use retargeting in code

```python
import sys
sys.path.insert(0, "sharpa_teleop")

import numpy as np
from retargeting import DittoToSharpaRetargeter

retargeter = DittoToSharpaRetargeter()
result = retargeter.retarget(ditto_q, sharpa_q_seed)
sharpa_q = result.sharpa_q
```

`ditto_q` — 7 leader joints (URDF order). `sharpa_q_seed` — full Sharpa `q`
(inactive fingers stay at seed).
