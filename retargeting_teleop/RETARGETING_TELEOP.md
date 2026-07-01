# Retargeting teleop

Maps Ditto leader index/thumb **fingerpad poses** to Sharpa index/thumb pads
(kinematic IK), with optional **force rendering** (haptics) back to the Ditto
leader. Per finger you can pick how **position** is driven (IK or direct joint
map) and where the rendered **force** comes from (estimate, tactile, or the
measured Sharpa joint current).

Install + env: see the [repo README](../README.md). Run everything from the
`sharpa_teleop/` repo root with `conda activate sharpa_ditto`. Before any real
Ditto hardware: `dynamixel-port --latency-timer 1`.

See also: [`docs/COMMANDS.md`](docs/COMMANDS.md) (copy-paste demo commands),
[`docs/FORCE_RENDERING.md`](docs/FORCE_RENDERING.md) and
[`docs/TELEOP_EQS.md`](docs/TELEOP_EQS.md) (the math + mode options).

## Viewer (no hardware)

URDFs, joint sliders, live Ditto → Sharpa retargeting:

```bash
python retargeting_teleop/viz/view_assets.py                 # both hands
python retargeting_teleop/viz/view_assets.py --leader-only   # Ditto only
python retargeting_teleop/viz/view_assets.py --sharpa-only   # Sharpa only
python retargeting_teleop/viz/view_assets.py --3f            # 3-finger Ditto URDF
```

## Teleop with viewer (hardware opt-in)

Hardware is **off by default**. `--ditto` drives the viewer from the physical
Ditto encoders (torque off); `--sharpa` streams retargeted joints to the Sharpa
Wave hand. With `--sharpa` the viewer draws pad-force arrows (Sharpa + Ditto
pads) and the would-be leader joint-torque arrows (`Jᵀ·F`, shown not commanded).

```bash
python retargeting_teleop/viz/view_teleop.py                  # viewer only
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa # both hardware
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa hand_config=ditto_hand_tactile
python retargeting_teleop/viz/view_teleop.py --ditto u2d2.fake_u2d2=true  # no Ditto USB
```

| Flag | Effect |
| --- | --- |
| `--ditto` | Enable Ditto leader hardware (encoders drive the viewer) |
| `--sharpa` | Enable Sharpa follower hardware (stream retargeted joints) |
| `--leader-only` / `--sharpa-only` | Show a single hand (no hardware) |
| `--3f` | Use the 3-finger Ditto leader URDF (index + middle + thumb) |

The control mode is **config-only** (`hand_config.control.fingers`); a
tactile/mix config auto-enables `--sharpa`. The viewer still has a live **Force
source** dropdown for interactive comparison. Each interface also has a runtime
checkbox in the **Controls** GUI folder.

## Headless teleop (no viewer)

Same core loop as the viewer, without Viser. At least one of `--ditto` /
`--sharpa` is required.

```bash
python retargeting_teleop/run_teleop.py --ditto --sharpa
python retargeting_teleop/run_teleop.py --ditto --sharpa --no-thumb   # index only
python retargeting_teleop/run_teleop.py --ditto --rate 200 --retarget-rate 40
```

`--rate` is the cheap leader-poll/force-loop rate; `--retarget-rate` is the
expensive IK rate, kept low (~40 Hz) on purpose — the IK shares the GIL with
finger_aloha's 200 Hz read thread, so running it at full rate starves that
thread (and the Sharpa hand can't track faster anyway).

The Viser-free core lives in `teleop/engine.py` (`RetargetTeleopEngine`:
`poll_leader()`, `retarget()`, `estimate_force_feedback()`); the GUI and headless
runners drive the same engine.

## Force rendering (haptics)

There is no 1:1 joint mapping, so force feedback is **model-based**: from the
Sharpa contact we get a pad force, map it to would-be Ditto joint torques via
`Jᵀ`, then feed those as synthetic follower currents through the exact same
finger_aloha `CurrentController` force-rendering + damping law as `joint_teleop`.
The measured-current mode skips the model and feeds the raw Sharpa joint torque
straight through (true `joint_teleop`-style, requires a joint map).

The control mode (per-finger position/force + force source) is fully defined by
the chosen config; a per-finger summary is printed at startup. See
[`docs/COMMANDS.md`](docs/COMMANDS.md) for the full demo set.

```bash
# headless force render (mode comes from the config):
python retargeting_teleop/run_force_render.py hand_config=ditto_hand_tactile
# add the Sharpa hand for an estimate config:
python retargeting_teleop/run_force_render.py hand_config=ditto_index_force_render --sharpa
# raise a per-joint gain at runtime (index PIP = joint_settings index 2):
python retargeting_teleop/run_force_render.py hand_config=ditto_hand_tactile \
  hand_config.leader.joint_settings.2.current_control.force_rendering_gain=0.05
```

### Configs (`conf/hand_config/`)

| Config | Fingers | Position | Force |
| --- | --- | --- | --- |
| `ditto_7dof_leader_only` | index+thumb | retarget | none (viewer/teleop default) |
| `ditto_index_force_render` | index | retarget | estimate |
| `ditto_thumb_force_render` | thumb | retarget | estimate |
| `ditto_index_tactile` | index | retarget | tactile |
| `ditto_thumb_tactile` | thumb | retarget | tactile |
| `ditto_hand_tactile` | index+thumb | retarget | tactile |
| `ditto_hand_mixed` | index+thumb | index=joint, thumb=retarget | index=measured, thumb=tactile |
| `ditto_hand_mixed_ik` | index+thumb | retarget | index=measured, thumb=tactile |

Per-config tuning (gains, `torque_to_mA`, thresholds, EMA `force_rendering_alpha`
/ `torque_filter_alpha`, damping) lives under `leader.joint_settings.*` and
`force_render.joints`. Estimate and tactile are **independent tuning surfaces**
(sensor scale ≠ model estimate). All values mirror
`joint_teleop/conf/hand_config/sharpa_3dof_index.yaml` — keep them in sync by
hand.

### Control modes (per finger)

Each finger declares **two independent choices** in one place,
`hand_config.control.fingers`: how its **position** is driven and where its
**force feedback** comes from. This is the single source of truth — there are no
force-source CLI flags, and the startup banner echoes the resolved mode.

**Position** — how the Sharpa finger is driven:

| `position` | Meaning |
| --- | --- |
| `retarget` | Cartesian IK: match the Ditto fingerpad pose (the normal mode) |
| `joint` | direct leader→Sharpa joint map (1:1, `joint_teleop`-style); needs `control.joint_map` |

**Force** — where the haptic feedback rendered to the leader comes from:

| `force` | Meaning |
| --- | --- |
| `estimate` | task-space: model-based `Jᵀ` solve of contact force from Sharpa joint torques |
| `tactile` | task-space: fingertip tactile F6 sensor, rotated into the Sharpa base frame |
| `mix` | task-space: 50/50 blend of `estimate` + `tactile` |
| `measured` | joint-space: raw measured Sharpa joint current (`joint_teleop`-style); needs `control.joint_map` |

The `estimate`/`tactile`/`mix` sources all output force in the same Sharpa-base
frame at the pad, so they are interchangeable per finger (you can run `index:
estimate`, `thumb: tactile`). `measured` skips the model and feeds the Sharpa
joint torque straight through. Index supports every combination; the thumb is
intended for `retarget` position.

Two shortcut strings exist: `retarget` == `{position: retarget, force: estimate}`
and `joint` == `{position: joint, force: measured}`.

```yaml
control:
  fingers:
    index: {position: joint,    force: measured}   # 1:1 joint map + joint-current force
    thumb: {position: retarget, force: tactile}    # IK position + tactile force
  # Required for any finger using position=joint and/or force=measured. `scale`
  # sets the sign for BOTH position (θ_sharpa = scale·θ_ditto) and force
  # (τ_ditto = scale·τ_sharpa).
  joint_map:
    index:
    - {ditto_joint: index_joint_0, sharpa_joint: right_index_MCP_AA, scale: -1.0}
    - {ditto_joint: index_joint_1, sharpa_joint: right_index_MCP_FE, scale: -1.0}
    - {ditto_joint: index_joint_2, sharpa_joint: right_index_PIP,    scale: -1.0}
```

Tactile/mix sources auto-enable the Sharpa follower. Two more config-only
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
python retargeting_teleop/viz/force_plot.py hand_config=ditto_hand_tactile --sharpa
python retargeting_teleop/viz/force_plot.py                                   # leader only
```

Left column: estimated joint torque (Nm), raw vs filtered. Right column: force
rendering vs damping vs net rendered current vs measured leader current (mA),
deadband in red. Joints with `plot_joint: false` in the config are skipped.

## Configs

- Ditto: `conf/hand_config/*.yaml` (default `ditto_7dof_leader_only`). Override
  with `hand_config=...`. `motor_models` / `joint_configs` come from
  `finger_aloha` via the Hydra searchpath.
- Sharpa: `conf/sharpa/default.yaml` (serial, enabled joints, speed/current
  coeffs, IO rate). `angle_overrides_deg` widens the SDK position-clamp ROM per
  joint without editing the `sharpa_controller` submodule.

Hardware adapters in `hardware_interfaces/`: `ditto_leader/` (encoder reads),
`sharpa_follower/` (sends retargeted `sharpa_q`; `conventions.py` maps URDF →
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
sys.path.insert(0, "retargeting_teleop")

import numpy as np
from retargeting import DittoToSharpaRetargeter

retargeter = DittoToSharpaRetargeter()
result = retargeter.retarget(ditto_q, sharpa_q_seed)
sharpa_q = result.sharpa_q
```

`ditto_q` — 7 leader joints (URDF order). `sharpa_q_seed` — full Sharpa `q`
(inactive fingers stay at seed).
