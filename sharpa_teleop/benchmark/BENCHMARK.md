# Ditto leader read benchmarks

Hardware diagnostics for the Ditto Dynamixel bus (U2D2). Use this when adding
motors or chasing sync-read timeouts (`-3001` / `COMM_RX_TIMEOUT`) in teleop.

**Script:** `diagnose_ditto_reads.py` — self-contained motor ID lists, no teleop
config required.

Run from the `sharpa_teleop` repo root:

```bash
python sharpa_teleop/benchmark/diagnose_ditto_reads.py <mode> [options]
```

Default port/baud: `/dev/ttyUSB0` @ 4 Mbps. Override with `--port` / `--baud`.

## Motor IDs (built into the script)

| Chain | IDs (in order) |
|-------|----------------|
| 2-finger (7 motors) | 121, 122, 123, 111, 112, 113, 114 |
| 3-finger (10 motors) | 121, 122, 123, **131, 132, 133**, 111, 112, 113, 114 |

Use `--3f` for the 10-motor chain, or `--motors 121,122,131` for a custom list.

## Sync read modes (teleop mapping)

| Benchmark label | `hand_config` `sync_read_mode` | Bytes/motor | Use when |
|-----------------|--------------------------------|-------------|----------|
| sync position | `position` | 4 | Position-only teleop (`torque_off`) |
| sync pos_vel | `pos_vel` | 8 | Force render + damping (velocity, no present current) |
| sync all states | `all` | 10 | Legacy full read; heavier than `pos_vel` for haptics |

Present current is not used in force rendering — prefer **`pos_vel`** over `all`
for current-mode leaders.

---

## Recommended workflow

### 1. `ping` — is each motor on the bus?

```bash
python sharpa_teleop/benchmark/diagnose_ditto_reads.py ping --3f
python sharpa_teleop/benchmark/diagnose_ditto_reads.py ping --3f --sync-read
```

**Expect:** every ID prints `OK` with model number and encoder position. Any `FAIL`
means wrong ID, no power, bad connector, or wrong USB port.

`--sync-read` tries one group sync read on all motors that pinged — quick check
that the group packet works before a longer benchmark.

Optional: `--scan` probes IDs 110–135.

---

### 2. `sweep` — latency vs motor count (main tool)

```bash
python sharpa_teleop/benchmark/diagnose_ditto_reads.py sweep --3f
```

Adds motors **one at a time** in chain order. At each count it benchmarks two
**read + write** loops (sync read then `sync_write_currents` with zeros — same
shape as force-render teleop):

| Step | Read API | What it models |
|------|----------|----------------|
| 1 | `sync_read_state()` | `sync_read_mode: all` (10 B/motor) |
| 2 | `sync_read_pos_vel()` | `sync_read_mode: pos_vel` (8 B/motor) |

Defaults: `warmup=20`, `iterations=100`. `--no-plot` for terminal-only output.

**Expect:**

- Printed table per step for both methods: `ok=%`, mean/p95 latency (ms), implied
  Hz, pass/fail vs 100 Hz and 200 Hz budgets.
- Plot saved to `sharpa_teleop/benchmark/diagnose_ditto_reads_sweep.png` —
  latency and rate vs motor count for both curves.
- Red `×` on the plot = sync failures at that count.

**How to read it:**

- Latency grows roughly linearly with motor count; `pos_vel` should sit between
  `position` and `all` (this sweep compares `all` vs `pos_vel` only).
- Sudden jump in p95 or `ok` dropping below ~99% → bus overloaded at that count;
  lower `control_frequency` or use a lighter read mode (`position` or `pos_vel`).
- `-3001` at 10 motors but clean at 7 → timing/packet size, not a dead motor.
- If `pos_vel` meets your target Hz but `all` does not, use `pos_vel` in
  `hand_config` for force-render teleop.

---

### 3. `trials` — bad motor vs bus timing?

```bash
python sharpa_teleop/benchmark/diagnose_ditto_reads.py trials
python sharpa_teleop/benchmark/diagnose_ditto_reads.py trials --read-mode all
```

Runs fixed scenarios: baseline 7-motor, each middle motor alone (131/132/133),
baseline + one middle, baseline + all middle, full 10-motor teleop order.

Uses one read method for all trials: `sync position` (default) or `sync all`
(`--read-mode all`). Read only — no current write.

**Expect:**

- Baseline 7 should be ~100% success.
- One middle ID fails **alone** → suspect that motor or its wiring.
- All middle OK alone but fail when **added to baseline** → motor-count / bus
  timing (typical when expanding 7 → 10 motors).
- Summary line at the end interprets the pattern.

---

### 4. `methods` — compare read modes at full motor count

```bash
python sharpa_teleop/benchmark/diagnose_ditto_reads.py methods --3f
```

At the full motor set, compares **read only** (no write):

| Method | API | Bytes/motor |
|--------|-----|-------------|
| sync position | `sync_read_specific("position")` | 4 |
| sync all states | `sync_read_state()` | 10 |
| individual position | per-motor read packets | 4 × N round trips |

Defaults: `warmup=50`, `iterations=300`.

**Expect:** table with success rate and p95 latency; recommendation for 100 / 200 Hz.
Plot: `sharpa_teleop/diagnose_ditto_reads_methods.png` (cwd-relative default).

Use `sweep` for `pos_vel` vs `all` with write; `methods` does not include
`pos_vel` yet.

**Teleop picks:**

- `ditto_*_leader_only` / open-loop → `sync_read_mode: position`
- Force render (tactile / estimate) → `sync_read_mode: pos_vel`
- Avoid `all` unless you need present current for logging

---

## Teleop targets

| Rate | Budget (p95) |
|------|----------------|
| 100 Hz | &lt; 10 ms |
| 200 Hz | &lt; 5 ms |

`sweep` and `methods` mark ✓/✗ against these. Hitting 200 Hz at 10 motors on a
single daisy chain is tight; parallel branches or fewer motors per sync group
helps.

## Dependencies

- `dynamixel_u2d2` (local `ditto/dynamixel_u2d2` or `pip install -e`)
- `matplotlib` (for plots)
