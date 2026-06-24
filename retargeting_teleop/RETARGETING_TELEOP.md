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

## Ditto leader + viewer

Physical Ditto hand drives the viewer (leader-only, torque off):

```bash
dynamixel-port --latency-timer 1   # before real hardware

python retargeting_teleop/viz/view_teleop.py
python retargeting_teleop/viz/view_teleop.py u2d2.usb_port=/dev/ttyUSB0
python retargeting_teleop/viz/view_teleop.py u2d2.fake_u2d2=true   # no USB
```

Hand config: `conf/hand_config/ditto_7dof_leader_only.yaml` (default). Override with `hand_config=...` or edit that file. `motor_models` / `joint_configs` still come from `finger_aloha` via Hydra searchpath.

Hardware adapters live in `hardware_interfaces/` (`ditto_leader/` for encoder reads, `sharpa_follower/` for future Sharpa commands).

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
