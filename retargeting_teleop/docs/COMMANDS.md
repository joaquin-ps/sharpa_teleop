# Demo commands

Run all of these from the `sharpa_teleop/` directory. Headless teleop
(`run_teleop.py`, `run_force_render.py`) always connects Ditto + Sharpa.
`view_teleop.py` is viewer-only unless you pass `--ditto` and/or `--sharpa`.
The control mode is defined by `hand_config` — `run_force_render.py` prints a
per-finger summary at startup. Open the viewer at the URL it prints
(default http://localhost:8080).

## 1. Retargeting teleop, no force feedback (with visualizer)

```bash
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa
```

## 2. Index + thumb retargeting, tactile force on both (headless)

```bash
python retargeting_teleop/run_force_render.py hand_config=ditto_hand_tactile
```

## 3. Index + thumb retargeting, tactile force on thumb, joint-current force on index (headless)

```bash
python retargeting_teleop/run_force_render.py hand_config=ditto_hand_mixed_ik
```

## 4. Index + thumb retargeting, estimated force on both (headless)

```bash
python retargeting_teleop/run_force_render.py hand_config=ditto_hand_tactile \
  hand_config.control.fingers.index.force=estimate \
  hand_config.control.fingers.thumb.force=estimate
```

## 5. Visualizer: see estimated forces vs tactile forces

```bash
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa hand_config=ditto_hand_tactile
```
