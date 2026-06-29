# Demo commands

Run all of these from the `sharpa_teleop/` directory. `--ditto` enables the real
Ditto leader, `--sharpa` enables the real Sharpa follower. The control mode
(per-finger position/force + force source) is defined entirely by the chosen
`hand_config` — `run_force_render.py` prints a per-finger summary at startup.
Open the viewer at the URL it prints (default http://localhost:8080).

## 1. Retargeting teleop, no force feedback (with visualizer)

Reads the Ditto leader, retargets to the Sharpa hand, no haptics.

```bash
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa
```

## 2. Index + thumb retargeting, tactile force on both (headless)

```bash
python retargeting_teleop/run_force_render.py hand_config=ditto_hand_tactile --sharpa
```

## 3. Index + thumb retargeting, tactile force on thumb, joint-current force on index (headless)

```bash
python retargeting_teleop/run_force_render.py hand_config=ditto_hand_mixed_ik --sharpa
```

## 4. Index + thumb retargeting, estimated force on both (headless)

Override each finger's force source to the model-based Jᵀ estimate via Hydra
(no CLI flags — it's a config value):

```bash
python retargeting_teleop/run_force_render.py hand_config=ditto_hand_tactile \
  hand_config.control.fingers.index.force=estimate \
  hand_config.control.fingers.thumb.force=estimate --sharpa
```

## 5. Visualizer: see estimated forces vs tactile forces

Press on the Sharpa fingertips and watch the force arrows. The initial source
comes from the config; toggle the **Force source** dropdown in the viewer
between `estimate` and `tactile` to compare live.

```bash
python retargeting_teleop/viz/view_teleop.py --ditto --sharpa hand_config=ditto_hand_tactile
```
