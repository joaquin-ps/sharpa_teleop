# Demo commands

Run all of these from the `sharpa_teleop/` directory. Headless force render
(`run_force_render.py`) always connects Ditto + Sharpa. `view_teleop.py` is
viewer-only unless you pass `--ditto` and/or `--sharpa`. The control mode is
defined by `hand_config` — `run_force_render.py` prints a per-finger summary at
startup. Open the viewer at the URL it prints (default http://localhost:8080).

Top-level configs live in `conf/hand_config/*.yaml` (not `fingers/` or `old/`).

## 1. Retargeting teleop, no force feedback (with visualizer)

```bash
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa --3f
```

## 2. 2f / 3f tactile force render (headless)

```bash
python retargeting_teleop/run_force_render.py hand_config=ditto_2f_tactile
python retargeting_teleop/run_force_render.py hand_config=ditto_3f_tactile
```

## 3. 2f / 3f blended force sources (headless)

```bash
python retargeting_teleop/run_force_render.py hand_config=ditto_2f_blend
python retargeting_teleop/run_force_render.py hand_config=ditto_3f_blend
```

## 4. Visualizer with force-render config

```bash
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa hand_config=ditto_2f_tactile
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa --3f hand_config=ditto_3f_tactile
```
