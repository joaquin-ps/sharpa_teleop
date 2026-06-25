# Retargeting teleop

Maps Ditto leader index/thumb **fingerpad poses** to Sharpa index/thumb **retargeting pads** (kinematic IK, no force feedback yet).

Install and env setup: see the [repo README](../README.md).

## Viewer (no hardware)

URDFs, joint sliders, live Ditto → Sharpa retargeting:

```bash
conda activate sharpa_ditto
python retargeting_teleop/viz/view_assets.py
```

Options: `--leader-only`, `--sharpa-only`

## Hardware: Ditto leader + Sharpa follower

Hardware is **off by default** (viewer only). Opt in per interface: the
physical Ditto hand drives the viewer (leader-only, torque off) with `--ditto`,
and retargeted index/thumb joints stream to the physical Sharpa Wave hand with
`--sharpa`:

```bash
dynamixel-port --latency-timer 1   # before real Ditto hardware

python retargeting_teleop/viz/view_teleop.py                   # viewer only, no hardware
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa  # both hardware
python retargeting_teleop/viz/view_teleop.py --ditto u2d2.usb_port=/dev/ttyUSB0
python retargeting_teleop/viz/view_teleop.py --ditto u2d2.fake_u2d2=true  # no Ditto USB
```

Enable hardware interfaces independently:

| Flag | Effect |
| --- | --- |
| `--ditto` | Enable Ditto leader hardware (physical encoders drive Ditto) |
| `--sharpa` | Enable Sharpa follower hardware (stream retargeted joints to the hand) |
| (none) | Viewer only (same as `view_assets.py`) |

Each enabled interface also has a runtime checkbox in the **Controls** GUI folder
("Drive Ditto from leader hardware", "Send retargeting to Sharpa hardware").

Configs:
- Ditto: `conf/hand_config/ditto_7dof_leader_only.yaml` (default). Override with `hand_config=...`. `motor_models` / `joint_configs` come from `finger_aloha` via Hydra searchpath.
- Sharpa: `conf/sharpa/default.yaml` (serial, enabled joints, speed/current coeffs, IO rate). `angle_overrides_deg` widens the SDK position-clamp ROM per joint (e.g. more thumb travel) without editing the `sharpa_controller` submodule.

Hardware adapters live in `hardware_interfaces/`:
- `ditto_leader/` — encoder reads from the Ditto leader.
- `sharpa_follower/` — sends retargeted `sharpa_q` to the Sharpa Wave hand. `conventions.py` maps Sharpa URDF joints → SDK joints (and holds per-joint sign/offset calibration hooks); `session.py` owns the SDK connection.

## Headless teleop (no viewer)

Same core loop as `view_teleop.py` but without Viser — read the Ditto leader,
retarget, stream to the Sharpa follower, and log estimated pad forces +
would-be leader joint torques. At least one of `--ditto` / `--sharpa` is
required (there is nothing to render headlessly):

```bash
python retargeting_teleop/run_teleop.py --ditto --sharpa
python retargeting_teleop/run_teleop.py --ditto u2d2.usb_port=/dev/ttyUSB0
python retargeting_teleop/run_teleop.py --ditto --rate 200 --retarget-rate 40
```

`--rate` is the cheap leader-poll / force-loop rate; `--retarget-rate` is the
expensive IK rate. They are **decoupled on purpose**: the retargeting IK
(~1.7 ms with `IK_STREAM`) shares the GIL with finger_aloha's 200 Hz read
thread, so running it at full poll rate starves that thread. Keep
`--retarget-rate` modest (~40 Hz — the Sharpa hand can't track faster anyway).

The shared, Viser-free core lives in `teleop/engine.py`
(`RetargetTeleopEngine`): it owns the retargeter plus the hardware session
references and exposes `poll_leader()`, `retarget()`, and
`estimate_force_feedback()`. Both the GUI viewer and the headless runner drive
this same engine, so their behavior stays in sync.

## Viewer quick reference

- **Green** — `retarget_base` on each hand
- **Cyan / orange (on mesh)** — achieved Ditto / Sharpa pad frames
- **Pale cyan / orange (floating)** — Sharpa IK targets
- GUI folders — cartesian scale and position/orientation IK weights per finger

Tune defaults in `retargeting/retargeter.py` (`index_cartesian_scale`, `thumb_*_weight`, etc.).

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

`ditto_q` — 7 leader joints (URDF order). `sharpa_q_seed` — full Sharpa `q` (inactive fingers stay at seed).
