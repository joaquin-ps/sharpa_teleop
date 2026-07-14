# Demo commands

Run all of these from the repo root. Headless teleop (`run_teleop.py`) always
connects Ditto + Sharpa. `view_teleop.py` is viewer-only unless you pass
`--ditto` and/or `--sharpa`. The control mode is defined by `hand_config` —
`run_teleop.py` prints a per-finger summary at startup. Open the viewer at the
URL it prints (default http://localhost:8080).

Top-level configs live in `conf/hand_config/*.yaml`; joint-only bring-up configs
are under `conf/hand_config/joint/` (not `fingers/` or `old/`).

## 1. Retargeting teleop, no force feedback (with visualizer)

```bash
python sharpa_teleop/viz/view_teleop.py --ditto --sharpa
```

```bash
python sharpa_teleop/viz/view_teleop.py --ditto --sharpa --3f
```

## 2. 2f / 3f tactile force render (headless)

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_2f_tactile
```

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_3f_tactile
```

## 3. 2f / 3f tactile + vibration haptics (headless)

Uses `ditto_haptics/config/thumb_index.yaml` (vib motors on index + thumb).

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_2f_tactile_haptics
```

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_3f_tactile_haptics
```

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_2f_tactile_haptics \
  hand_config.haptics.vib_port=/dev/ttyACM1
```

## 4. 2f / 3f blended force sources (headless)

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_2f_blend
```

```bash
python sharpa_teleop/run_teleop.py hand_config=ditto_3f_blend
```      

## 5. Direct joint map (headless)

```bash
python sharpa_teleop/run_teleop.py hand_config=joint/sharpa_3dof_index
```

```bash
python sharpa_teleop/run_teleop.py hand_config=joint/sharpa_3dof_middle
```

## 6. Visualizer with force-render config

```bash
python sharpa_teleop/viz/view_teleop.py --ditto --sharpa hand_config=ditto_2f_tactile
```

```bash
python sharpa_teleop/viz/view_teleop.py --ditto --sharpa --3f hand_config=ditto_3f_tactile
```
