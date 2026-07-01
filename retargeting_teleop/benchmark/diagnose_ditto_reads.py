#!/usr/bin/env python3
"""Ditto leader Dynamixel read diagnostics: connectivity, latency, and sync errors.

One tool for bringing up new motors on the bus:

  ping     Ping each motor; optional group sync read on responders
  sweep    Add motors one-by-one; compare sync all vs pos_vel read+write (current=0)
  methods  Compare read methods at a fixed motor set (sync pos / sync all / individual)
  trials   Isolate sync failures (baseline 7-motor + each new middle motor)

Usage (from sharpa_teleop repo root):
    python retargeting_teleop/benchmark/diagnose_ditto_reads.py ping --3f
    python retargeting_teleop/benchmark/diagnose_ditto_reads.py sweep --3f
    python retargeting_teleop/benchmark/diagnose_ditto_reads.py methods --3f
    python retargeting_teleop/benchmark/diagnose_ditto_reads.py trials
    python retargeting_teleop/benchmark/diagnose_ditto_reads.py sweep --motors 121,122,131
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

_BENCHMARK = Path(__file__).resolve().parent
_REPO = _BENCHMARK.parents[1]  # sharpa_teleop (for finger_aloha / dynamixel_u2d2)
for _path in (_REPO / "finger_aloha", _REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import matplotlib.pyplot as plt  # noqa: E402
from dynamixel_sdk import COMM_SUCCESS  # noqa: E402
from dynamixel_u2d2 import U2D2Interface  # noqa: E402
from dynamixel_u2d2.u2d2_interface import ADDR_PRESENT_POSITION  # noqa: E402

# --- Ditto leader motor chains (Dynamixel IDs on the U2D2 bus) ----------------
# 7-DoF: index 121-123, thumb 111-114.
DITTO_2F_MOTOR_IDS: tuple[int, ...] = (121, 122, 123, 111, 112, 113, 114)
DITTO_2F_JOINT_NAMES: tuple[str, ...] = (
    "index_joint_0",
    "index_joint_1",
    "index_joint_2",
    "thumb_joint_0",
    "thumb_joint_1",
    "thumb_joint_2",
    "thumb_joint_3",
)

# 10-DoF: index 121-123, middle 131-133, thumb 111-114 (teleop / hand_config order).
DITTO_3F_MOTOR_IDS: tuple[int, ...] = (
    121, 122, 123, 131, 132, 133, 111, 112, 113, 114,
)
DITTO_3F_JOINT_NAMES: tuple[str, ...] = (
    "index_joint_0",
    "index_joint_1",
    "index_joint_2",
    "middle_joint_0",
    "middle_joint_1",
    "middle_joint_2",
    "thumb_joint_0",
    "thumb_joint_1",
    "thumb_joint_2",
    "thumb_joint_3",
)

TARGET_HZ = (100, 200, 300)
_ADDR_PRESENT_POSITION = ADDR_PRESENT_POSITION

# Original 2-finger leader chain (before middle finger).
BASELINE_7: tuple[int, ...] = DITTO_2F_MOTOR_IDS
MIDDLE_NEW: tuple[int, ...] = (131, 132, 133)


@dataclass
class ReadStats:
    label: str
    motor_ids: tuple[int, ...]
    method: str
    attempts: int
    successes: int
    failures: int
    mean_ms: float
    median_ms: float
    p95_ms: float
    max_ms: float

    @property
    def num_motors(self) -> int:
        return len(self.motor_ids)

    @property
    def added_motor_id(self) -> int:
        return self.motor_ids[-1] if self.motor_ids else -1

    @property
    def success_rate(self) -> float:
        return self.successes / self.attempts if self.attempts else 0.0

    @property
    def mean_hz(self) -> float:
        return 1000.0 / self.mean_ms if self.mean_ms == self.mean_ms and self.mean_ms > 0 else 0.0

    @property
    def p95_hz(self) -> float:
        return 1000.0 / self.p95_ms if self.p95_ms == self.p95_ms and self.p95_ms > 0 else 0.0

    def meets_hz(self, target: int) -> bool:
        budget_ms = 1000.0 / target
        return self.success_rate >= 0.99 and self.p95_ms == self.p95_ms and self.p95_ms < budget_ms


def _percentile(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * p
    f = int(k)
    c = min(f + 1, len(sorted_vals) - 1)
    if f == c:
        return sorted_vals[f]
    return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)


def _stats_from_latencies(
    latencies: list[float],
    *,
    label: str,
    motor_ids: tuple[int, ...],
    method: str,
    attempts: int,
    failures: int,
) -> ReadStats:
    if latencies:
        ordered = sorted(latencies)
        return ReadStats(
            label=label,
            motor_ids=motor_ids,
            method=method,
            attempts=attempts,
            successes=len(latencies),
            failures=failures,
            mean_ms=statistics.mean(latencies),
            median_ms=statistics.median(latencies),
            p95_ms=_percentile(ordered, 0.95),
            max_ms=max(latencies),
        )
    return ReadStats(
        label=label,
        motor_ids=motor_ids,
        method=method,
        attempts=attempts,
        successes=0,
        failures=failures,
        mean_ms=float("nan"),
        median_ms=float("nan"),
        p95_ms=float("nan"),
        max_ms=float("nan"),
    )


def _setup_iface(
    port: str,
    baud: int,
    motor_ids: list[int],
    read_mode: str,
) -> U2D2Interface:
    iface = U2D2Interface(port, baud, motor_ids=motor_ids, verbose=False)
    if read_mode == "position":
        iface.init_specific_group_sync_read("position")
    elif read_mode == "pos_vel":
        iface.init_group_sync_read_pos_vel(motor_ids)
    return iface


def _read_individual_position(iface: U2D2Interface, motor_ids: list[int]) -> None:
    for motor_id in motor_ids:
        _pos, result, _ = iface._packetHandler.read4ByteTxRx(
            iface._portHandler, motor_id, _ADDR_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            raise RuntimeError(f"Motor {motor_id} position read failed")


def _make_read_fn(
    method: str,
    motor_ids: list[int],
    *,
    with_write: bool = False,
) -> Callable[[U2D2Interface], None]:
    zero_currents = [0] * len(motor_ids)

    def _read(iface: U2D2Interface) -> None:
        if method == "sync_position":
            iface.sync_read_specific("position")
        elif method == "sync_all":
            iface.sync_read_state()
        elif method == "sync_pos_vel":
            iface.sync_read_pos_vel()
        elif method == "individual_pos":
            _read_individual_position(iface, motor_ids)
        else:
            raise ValueError(f"unknown method: {method}")
        if with_write:
            iface.sync_write_currents(zero_currents)

    return _read


def _benchmark(
    port: str,
    baud: int,
    motor_ids: list[int],
    method: str,
    *,
    label: str,
    warmup: int,
    iterations: int,
    with_write: bool = False,
) -> ReadStats:
    if method == "sync_position":
        iface = _setup_iface(port, baud, motor_ids, "position")
    elif method == "sync_all":
        iface = U2D2Interface(port, baud, motor_ids=motor_ids, verbose=False)
    elif method == "sync_pos_vel":
        iface = _setup_iface(port, baud, motor_ids, "pos_vel")
    elif method == "individual_pos":
        iface = U2D2Interface(port, baud, verbose=False)
    else:
        raise ValueError(method)

    read_fn = _make_read_fn(method, motor_ids, with_write=with_write)
    latencies: list[float] = []
    failures = 0
    try:
        for i in range(warmup + iterations):
            t0 = time.perf_counter()
            try:
                read_fn(iface)
                ok = True
            except Exception:
                ok = False
            if i < warmup:
                continue
            if ok:
                latencies.append((time.perf_counter() - t0) * 1000.0)
            else:
                failures += 1
    finally:
        iface.close()

    return _stats_from_latencies(
        latencies,
        label=label,
        motor_ids=tuple(motor_ids),
        method=method,
        attempts=iterations,
        failures=failures,
    )


def _print_stats(stats: ReadStats) -> None:
    ids = ",".join(str(m) for m in stats.motor_ids)
    if stats.successes == 0:
        print(
            f"  {stats.label:32s}  FAIL  "
            f"({stats.failures}/{stats.attempts} errors)  [{ids}]"
        )
        return
    hz_flags = "  ".join(
        f"{hz}Hz:{'✓' if stats.meets_hz(hz) else '✗'}" for hz in TARGET_HZ
    )
    print(
        f"  {stats.label:32s}  "
        f"ok={stats.success_rate * 100:5.1f}%  "
        f"mean={stats.mean_ms:5.2f} ms (~{stats.mean_hz:4.0f} Hz)  "
        f"p95={stats.p95_ms:5.2f} ms  fail={stats.failures}  "
        f"[{ids}]  {hz_flags}"
    )


def _resolve_motor_ids(args: argparse.Namespace) -> tuple[list[int], dict[int, str]]:
    if args.motors:
        motor_ids = [int(x.strip()) for x in args.motors.split(",") if x.strip()]
        return motor_ids, {m: f"motor_{m}" for m in motor_ids}
    if args.three_finger:
        return list(DITTO_3F_MOTOR_IDS), dict(
            zip(DITTO_3F_MOTOR_IDS, DITTO_3F_JOINT_NAMES, strict=True)
        )
    return list(DITTO_2F_MOTOR_IDS), dict(
        zip(DITTO_2F_MOTOR_IDS, DITTO_2F_JOINT_NAMES, strict=True)
    )


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=4_000_000)
    parser.add_argument("--3f", "--three-finger", action="store_true", dest="three_finger")
    parser.add_argument(
        "--motors",
        type=str,
        default=None,
        help="Comma-separated motor IDs (overrides --3f preset)",
    )


def _cmd_ping(args: argparse.Namespace) -> int:
    motor_ids, labels = _resolve_motor_ids(args)
    print(f"Port: {args.port}  Baud: {args.baud}")
    print(f"Pinging {len(motor_ids)} motor(s): {motor_ids}\n")

    iface = U2D2Interface(args.port, args.baud, verbose=False)
    reachable: list[int] = []
    try:
        print("Individual ping + position read:")
        print("-" * 72)
        for motor_id in motor_ids:
            joint = labels.get(motor_id, f"motor_{motor_id}")
            model, result, _ = iface._packetHandler.ping(iface._portHandler, motor_id)
            if result == COMM_SUCCESS:
                pos, pos_result, _ = iface._packetHandler.read4ByteTxRx(
                    iface._portHandler, motor_id, _ADDR_PRESENT_POSITION
                )
                pos_str = f"pos={int(pos)}" if pos_result == COMM_SUCCESS else "pos read failed"
                print(f"  OK   ID {motor_id:3d}  {joint:18s}  model={int(model)}  {pos_str}")
                reachable.append(motor_id)
            else:
                err = iface._packetHandler.getTxRxResult(result)
                print(f"  FAIL ID {motor_id:3d}  {joint:18s}  {err}")
        print("-" * 72)
        print(f"Reachable: {reachable or '(none)'}")

        if args.scan:
            print("\nScanning IDs 110-135...")
            found = iface.scan_motors_at_baudrate(args.baud, range(110, 136))
            print(f"  Found: {found or '(none)'}")

        if args.sync_read and reachable:
            print(f"\nGroup sync read on {reachable}...")
            reader = U2D2Interface(args.port, args.baud, motor_ids=reachable, verbose=True)
            try:
                reader.init_specific_group_sync_read("position")
                positions = reader.sync_read_specific("position")
                for mid, pos in zip(reachable, positions, strict=True):
                    print(f"  ID {mid}: pos={pos}")
            except Exception as exc:  # noqa: BLE001
                print(f"  Sync read failed: {exc}")
            finally:
                reader.close()
    finally:
        iface.close()
    return 0 if reachable else 1


def _plot_sweep(
    all_stats: list[ReadStats],
    pos_vel_stats: list[ReadStats],
    *,
    output: Path,
    baud: int,
    port: str,
) -> None:
    counts = [s.num_motors for s in all_stats]
    fig, (ax_lat, ax_hz) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    def _y(stats: list[ReadStats], field: str) -> list[float]:
        return [getattr(s, field) for s in stats]

    ax_lat.plot(counts, _y(all_stats, "mean_ms"), "o-", label="sync all mean", linewidth=2)
    ax_lat.plot(counts, _y(all_stats, "p95_ms"), "s--", label="sync all p95", alpha=0.8)
    ax_lat.plot(counts, _y(pos_vel_stats, "mean_ms"), "^-", label="sync pos_vel mean", linewidth=2)
    ax_lat.plot(counts, _y(pos_vel_stats, "p95_ms"), "v--", label="sync pos_vel p95", alpha=0.8)

    for stats in all_stats:
        if stats.success_rate > 0 and stats.mean_ms == stats.mean_ms:
            ax_lat.annotate(
                str(stats.added_motor_id),
                (stats.num_motors, stats.mean_ms),
                textcoords="offset points",
                xytext=(5, 4),
                fontsize=8,
                color="0.25",
            )
        elif stats.failures > 0:
            ax_lat.scatter(
                [stats.num_motors], [0.5], marker="x", s=120, color="red", zorder=5
            )
            ax_lat.annotate(
                str(stats.added_motor_id),
                (stats.num_motors, 0.5),
                textcoords="offset points",
                xytext=(8, 6),
                fontsize=8,
                color="red",
            )

    for hz in TARGET_HZ:
        ax_lat.axhline(1000 / hz, linestyle="--", alpha=0.4, label=f"{hz} Hz budget")
    ax_lat.set_ylabel("Read+write latency (ms)")
    ax_lat.set_title(
        f"Ditto sync all vs pos_vel read+write (current=0) "
        f"({port} @ {baud // 1_000_000} Mbps)"
    )
    ax_lat.grid(True, alpha=0.3)
    ax_lat.legend(loc="upper left", fontsize=8)

    ax_hz.plot(counts, _y(all_stats, "mean_hz"), "o-", label="sync all mean Hz", linewidth=2)
    ax_hz.plot(counts, _y(pos_vel_stats, "mean_hz"), "^-", label="sync pos_vel mean Hz", linewidth=2)
    for hz in TARGET_HZ:
        ax_hz.axhline(hz, linestyle="--", alpha=0.5)
    ax_hz.set_xlabel("Number of motors")
    ax_hz.set_ylabel("Rate (Hz)")
    ax_hz.grid(True, alpha=0.3)
    ax_hz.legend(loc="upper right", fontsize=8)
    ax_hz.set_xticks(counts)

    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"\nPlot saved: {output}")


def _cmd_sweep(args: argparse.Namespace) -> int:
    progression, _ = _resolve_motor_ids(args)
    print(f"Port: {args.port}  Baud: {args.baud}")
    print(f"Motor progression: {progression}")
    print(
        f"Read+write: sync all + sync pos_vel (current=0)  "
        f"warmup={args.warmup}  iterations={args.iterations}\n"
    )

    all_results: list[ReadStats] = []
    pos_vel_results: list[ReadStats] = []

    for n in range(1, len(progression) + 1):
        motor_ids = progression[:n]
        label = f"n={n} +{motor_ids[-1]}"
        print(f"=== {label} ===")
        all_stats = _benchmark(
            args.port,
            args.baud,
            motor_ids,
            "sync_all",
            label=f"{label} sync all",
            warmup=args.warmup,
            iterations=args.iterations,
            with_write=True,
        )
        all_results.append(all_stats)
        _print_stats(all_stats)

        pos_vel_stats = _benchmark(
            args.port,
            args.baud,
            motor_ids,
            "sync_pos_vel",
            label=f"{label} sync pos_vel",
            warmup=args.warmup,
            iterations=args.iterations,
            with_write=True,
        )
        pos_vel_results.append(pos_vel_stats)
        _print_stats(pos_vel_stats)
        print()

    if not args.no_plot:
        _plot_sweep(
            all_results,
            pos_vel_results,
            output=args.output,
            baud=args.baud,
            port=args.port,
        )
    return 0


def _plot_methods(results: list[ReadStats], *, output: Path, port: str, baud: int) -> None:
    labels = [r.label for r in results]
    x = range(len(results))
    fig, (ax_lat, ax_hz) = plt.subplots(1, 2, figsize=(10, 4))
    means = [r.mean_ms if r.mean_ms == r.mean_ms else 0 for r in results]
    p95s = [r.p95_ms if r.p95_ms == r.p95_ms else 0 for r in results]
    ax_lat.bar([i - 0.15 for i in x], means, width=0.3, label="mean")
    ax_lat.bar([i + 0.15 for i in x], p95s, width=0.3, label="p95")
    for hz in TARGET_HZ:
        ax_lat.axhline(1000 / hz, linestyle="--", alpha=0.5, label=f"{hz} Hz budget")
    ax_lat.set_xticks(list(x), labels, rotation=15, ha="right")
    ax_lat.set_ylabel("Latency (ms)")
    ax_lat.legend(fontsize=8)
    ax_lat.grid(True, alpha=0.3, axis="y")
    rates = [r.p95_hz if r.p95_hz > 0 else 0 for r in results]
    ax_hz.bar(x, rates)
    for hz in TARGET_HZ:
        ax_hz.axhline(hz, linestyle="--", alpha=0.5)
    ax_hz.set_xticks(list(x), labels, rotation=15, ha="right")
    ax_hz.set_ylabel("Hz (from p95)")
    ax_hz.grid(True, alpha=0.3, axis="y")
    fig.suptitle(f"Ditto read methods ({port} @ {baud // 1_000_000} Mbps, {results[0].num_motors} motors)")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"\nPlot saved: {output}")


def _cmd_methods(args: argparse.Namespace) -> int:
    motor_ids, _ = _resolve_motor_ids(args)
    methods = (
        ("sync_position", "sync position"),
        ("sync_all", "sync all states"),
        ("individual_pos", "individual position"),
    )
    print(f"Port: {args.port}  Baud: {args.baud}")
    print(f"Motors ({len(motor_ids)}): {motor_ids}\n")

    results: list[ReadStats] = []
    for method, label in methods:
        print(f"Running {label}...")
        results.append(
            _benchmark(
                args.port,
                args.baud,
                motor_ids,
                method,
                label=label,
                warmup=args.warmup,
                iterations=args.iterations,
            )
        )
        _print_stats(results[-1])

    viable = [r for r in results if r.success_rate >= 0.99 and r.p95_ms == r.p95_ms]
    print("\n--- Recommendation ---")
    if not viable:
        print("No method reached 99% success — run `ping` and check USB/power.")
    else:
        for target in sorted(TARGET_HZ, reverse=True):
            candidates = [r for r in viable if r.meets_hz(target)]
            if candidates:
                best = min(candidates, key=lambda r: r.p95_ms)
                print(
                    f"For {target} Hz: {best.label} "
                    f"(p95={best.p95_ms:.2f} ms, ~{best.p95_hz:.0f} Hz)."
                )
                break
        else:
            best = min(viable, key=lambda r: r.p95_ms)
            print(f"200 Hz not met. Best: {best.label} (p95={best.p95_ms:.2f} ms).")

    if not args.no_plot:
        _plot_methods(results, output=args.output, port=args.port, baud=args.baud)
    return 0


def _cmd_trials(args: argparse.Namespace) -> int:
    trials: list[tuple[str, tuple[int, ...]]] = [
        ("baseline 7 (index+thumb)", BASELINE_7),
        ("middle 131 alone", (131,)),
        ("middle 132 alone", (132,)),
        ("middle 133 alone", (133,)),
        ("baseline + 131", BASELINE_7 + (131,)),
        ("baseline + 132", BASELINE_7 + (132,)),
        ("baseline + 133", BASELINE_7 + (133,)),
        ("baseline + all middle", BASELINE_7 + MIDDLE_NEW),
        ("full 10 (teleop order)", DITTO_3F_MOTOR_IDS),
    ]
    method = "sync_position" if args.read_mode == "position" else "sync_all"
    print(f"Port: {args.port}  Baud: {args.baud}")
    print(f"Read mode: {args.read_mode}  iterations={args.iterations}\n")
    print("Interpretation:")
    print("  • baseline 7 should be ~100%")
    print("  • one middle ID fails alone → suspect that motor/wiring")
    print("  • all middle OK alone but fail when added → bus timing / motor count\n")

    results: list[ReadStats] = []
    for label, motor_ids in trials:
        print(f"Running: {label}...")
        stats = _benchmark(
            args.port,
            args.baud,
            list(motor_ids),
            method,
            label=label,
            warmup=args.warmup,
            iterations=args.iterations,
        )
        results.append(stats)
        _print_stats(stats)

    baseline = results[0]
    if baseline.success_rate < 0.99:
        print("\n⚠️  Baseline 7-motor sync already failing — check USB, power, or port.")
        return 1

    singles = results[1:4]
    adds = results[4:7]
    bad_singles = [r.label for r in singles if r.success_rate < 0.99]
    bad_adds = [r.label for r in adds if r.success_rate < 0.99]

    print("\n--- Summary ---")
    if bad_singles:
        print(f"⚠️  Motor(s) fail alone: {bad_singles}")
    if len(bad_adds) == 3 and not bad_singles:
        print("→ Middle motors OK alone but fail on baseline — likely bus timing at higher motor count.")
        print("  Use sync_read_mode: position and keep control_frequency ≤ what sweep shows is sustainable.")
    elif len(bad_adds) == 1:
        motor = bad_adds[0].split()[-1]
        print(f"→ Only adding motor {motor} breaks sync — check ID {motor} and harness.")
    elif not bad_adds:
        full = [r for r in results if r.num_motors == 10]
        if full and full[0].success_rate < 0.99:
            print("→ Per-motor adds OK; full 10-motor group still flaky — bus timing.")
        else:
            print("✅ All trials passed.")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    ping = sub.add_parser("ping", help="Ping each motor independently")
    _add_common_args(ping)
    ping.add_argument("--scan", action="store_true", help="Scan IDs 110-135")
    ping.add_argument("--sync-read", action="store_true", help="Try group sync on responders")
    ping.set_defaults(func=_cmd_ping)

    sweep = sub.add_parser("sweep", help="Latency vs motor count (default workflow)")
    _add_common_args(sweep)
    sweep.add_argument("--warmup", type=int, default=20)
    sweep.add_argument("--iterations", type=int, default=100)
    sweep.add_argument(
        "--output",
        type=Path,
        default=_BENCHMARK / "diagnose_ditto_reads_sweep.png",
    )
    sweep.add_argument("--no-plot", action="store_true")
    sweep.set_defaults(func=_cmd_sweep)

    methods = sub.add_parser("methods", help="Compare read methods at full motor set")
    _add_common_args(methods)
    methods.add_argument("--warmup", type=int, default=50)
    methods.add_argument("--iterations", type=int, default=300)
    methods.add_argument(
        "--output",
        type=Path,
        default=Path("retargeting_teleop/diagnose_ditto_reads_methods.png"),
    )
    methods.add_argument("--no-plot", action="store_true")
    methods.set_defaults(func=_cmd_methods)

    trials = sub.add_parser("trials", help="Baseline + middle-motor sync isolation")
    trials.add_argument("--port", default="/dev/ttyUSB0")
    trials.add_argument("--baud", type=int, default=4_000_000)
    trials.add_argument("--warmup", type=int, default=20)
    trials.add_argument("--iterations", type=int, default=200)
    trials.add_argument(
        "--read-mode",
        choices=("position", "all"),
        default="position",
    )
    trials.set_defaults(func=_cmd_trials)

    args = parser.parse_args()
    raise SystemExit(args.func(args))


if __name__ == "__main__":
    main()
