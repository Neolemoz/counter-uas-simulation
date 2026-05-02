#!/usr/bin/env python3
"""Monte Carlo harness: run N scenarios with varying seeds and report a credibility-grade summary.

This is the Phase 4 V&V driver.  ``analyze_run.parse_run_to_result`` already extracts
``success`` / ``miss_distance_m`` / ``intercept_time_s`` from a single log; this script wraps
that with a parameter sweep so we can produce the metrics a defense reviewer expects:

* success rate (count + percentage)
* miss distance: mean, median, P95, full empirical CDF
* intercept time: mean, std, P95
* per-run table (CSV) + aggregate JSON

Two operating modes
-------------------

1. ``--mode aggregate`` — point the script at a directory of existing logs and compute
   aggregate metrics.  No simulator is launched.  Useful for re-analysing past runs and for
   tests / CI where Gazebo is not available.

2. ``--mode run`` — drive ``scripts/run_capture.py`` for ``--n`` runs.  Each run gets a
   different ``seed`` injected into the launch arguments (``noise_seed:=<i>``); the rest of
   the launch args you pass on the command line are forwarded verbatim, so this is the same
   surface you already use for one-off runs.

Examples
--------

::

    # Reanalyse 30 logs already on disk:
    python3 scripts/monte_carlo.py --mode aggregate \
        --logs-dir runs/logs --pattern '*.log' --label baseline

    # Run 20 fresh sims with the predictive guidance + delay compensation enabled:
    python3 scripts/monte_carlo.py --mode run --n 20 --timeout-s 18 \
        --launch-args 'use_noisy_measurement:=true noise_std_m:=0.5' \
        --label predict_delay_off

    # Compare baseline vs variant (ablation): run two MC sets and print a diff table.
    python3 scripts/monte_carlo.py --mode run --n 20 --label baseline \
        --launch-args 'measurement_delay_s:=0.0'
    python3 scripts/monte_carlo.py --mode run --n 20 --label delay_compensated \
        --launch-args 'measurement_delay_s:=0.15'
    python3 scripts/monte_carlo.py --mode compare \
        --inputs runs/mc/baseline.json runs/mc/delay_compensated.json
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import statistics
import subprocess
import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_OUT = WORKSPACE / "runs" / "mc"


def _load_analyze_run():  # noqa: ANN201
    """Import scripts/analyze_run.py without polluting sys.path."""
    path = WORKSPACE / "scripts" / "analyze_run.py"
    spec = importlib.util.spec_from_file_location("analyze_run", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


# --------------------------------------------------------------------------------------
# Statistics
# --------------------------------------------------------------------------------------


def _percentile(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    if len(s) == 1:
        return s[0]
    idx = (len(s) - 1) * p / 100.0
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (idx - lo)


def _empirical_cdf(xs: list[float], n_points: int = 21) -> list[tuple[float, float]]:
    """Return ``(value, cumulative_fraction)`` pairs, sampled at evenly spaced quantiles."""
    if not xs:
        return []
    s = sorted(xs)
    out: list[tuple[float, float]] = []
    for i in range(n_points):
        q = i / (n_points - 1)
        idx = int(round(q * (len(s) - 1)))
        out.append((s[idx], (idx + 1) / len(s)))
    return out


def _summarise(results: list[dict], label: str) -> dict:
    """Aggregate per-run dicts (output of ``parse_run_to_result``) into a single summary."""
    n = len(results)
    successes = [r for r in results if r.get("success")]
    miss = [
        float(r["miss_distance_m"])
        for r in results
        if r.get("miss_distance_m") is not None and math.isfinite(float(r["miss_distance_m"]))
    ]
    times = [
        float(r["intercept_time_s"])
        for r in results
        if r.get("intercept_time_s") is not None and math.isfinite(float(r["intercept_time_s"]))
    ]
    summary: dict = {
        "label": label,
        "n_runs": n,
        "n_success": len(successes),
        "success_rate": (len(successes) / n) if n else float("nan"),
        "miss_distance_m": {
            "n": len(miss),
            "mean": statistics.fmean(miss) if miss else float("nan"),
            "median": statistics.median(miss) if miss else float("nan"),
            "stdev": statistics.pstdev(miss) if len(miss) > 1 else float("nan"),
            "p50": _percentile(miss, 50.0),
            "p90": _percentile(miss, 90.0),
            "p95": _percentile(miss, 95.0),
            "min": min(miss) if miss else float("nan"),
            "max": max(miss) if miss else float("nan"),
            "cdf": _empirical_cdf(miss),
        },
        "intercept_time_s": {
            "n": len(times),
            "mean": statistics.fmean(times) if times else float("nan"),
            "stdev": statistics.pstdev(times) if len(times) > 1 else float("nan"),
            "p50": _percentile(times, 50.0),
            "p95": _percentile(times, 95.0),
        },
    }
    return summary


def _print_summary(summary: dict) -> None:
    sep = "=" * 66
    print(sep)
    print(f"  MONTE CARLO SUMMARY  label={summary['label']}")
    print(sep)
    n = summary["n_runs"]
    ns = summary["n_success"]
    sr = summary["success_rate"]
    print(f"  Runs        : {n}")
    print(f"  Successes   : {ns} ({100.0 * sr:.1f}%)")
    md = summary["miss_distance_m"]
    print(
        f"  Miss dist   : n={md['n']}  mean={md['mean']:.3f} m  "
        f"median={md['median']:.3f} m  P95={md['p95']:.3f} m"
    )
    it = summary["intercept_time_s"]
    print(
        f"  Intercept t : n={it['n']}  mean={it['mean']:.3f} s  "
        f"P95={it['p95']:.3f} s"
    )
    print(sep)


def _print_compare(rows: list[dict]) -> None:
    if not rows:
        print("(no inputs to compare)")
        return
    headers = ["label", "n", "success%", "miss_mean", "miss_p95", "tint_mean"]
    fmt = "  {:<22s} {:>5s} {:>9s} {:>10s} {:>10s} {:>10s}"
    print(fmt.format(*headers))
    print("  " + "-" * 70)
    for s in rows:
        md = s["miss_distance_m"]
        it = s["intercept_time_s"]
        print(
            fmt.format(
                str(s["label"])[:22],
                str(s["n_runs"]),
                f"{100.0 * s['success_rate']:.1f}",
                f"{md['mean']:.3f}",
                f"{md['p95']:.3f}",
                f"{it['mean']:.3f}",
            )
        )


# --------------------------------------------------------------------------------------
# I/O
# --------------------------------------------------------------------------------------


def _write_outputs(out_dir: Path, label: str, summary: dict, rows: list[dict]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / f"{label}.json"
    csv_path = out_dir / f"{label}.csv"
    json_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    fields = ["run_id", "success", "miss_distance_m", "intercept_time_s", "layer_at_hit", "log_path"]
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in fields})
    print(f"[monte_carlo] wrote {json_path}")
    print(f"[monte_carlo] wrote {csv_path}")


# --------------------------------------------------------------------------------------
# Modes
# --------------------------------------------------------------------------------------


def cmd_aggregate(args: argparse.Namespace) -> int:
    analyze = _load_analyze_run()
    logs_dir = Path(args.logs_dir)
    if not logs_dir.is_dir():
        print(f"logs_dir not found: {logs_dir}", file=sys.stderr)
        return 2
    log_files = sorted(logs_dir.glob(args.pattern))
    if not log_files:
        print(f"no logs matched {args.pattern} in {logs_dir}", file=sys.stderr)
        return 2
    rows: list[dict] = []
    for log in log_files:
        result = analyze.parse_run_to_result(str(log))
        result["run_id"] = log.stem
        result["log_path"] = str(log)
        rows.append(result)
    summary = _summarise(rows, args.label)
    _print_summary(summary)
    _write_outputs(Path(args.out_dir), args.label, summary, rows)
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    analyze = _load_analyze_run()
    rc_script = WORKSPACE / "scripts" / "run_capture.py"
    if not rc_script.is_file():
        print(f"missing {rc_script}", file=sys.stderr)
        return 2

    rows: list[dict] = []
    base_args = args.launch_args or ""
    for i in range(args.n):
        seed = args.seed_base + i
        # Compose seed-aware launch args without overwriting whatever the caller already set.
        per_run_args = base_args
        if "noise_seed" not in base_args:
            per_run_args = f"{per_run_args} noise_seed:={seed}".strip()
        cmd = [
            sys.executable,
            str(rc_script),
            "--scenario", args.scenario,
            "--timeout-s", str(args.timeout_s),
            "--notes", f"mc_label={args.label} seed={seed}",
            "--launch-args", per_run_args,
        ]
        print(f"[monte_carlo] run {i + 1}/{args.n}  seed={seed}", flush=True)
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode not in (0, 124):
            print(r.stderr, file=sys.stderr)
            print(f"[monte_carlo] run failed (rc={r.returncode}); skipping", file=sys.stderr)
            continue
        out_lines = (r.stdout or "").strip().splitlines()
        if not out_lines:
            print("[monte_carlo] run produced no output; skipping", file=sys.stderr)
            continue
        log_path = Path(out_lines[0].strip())
        if not log_path.is_file():
            print(f"[monte_carlo] log path missing: {log_path}", file=sys.stderr)
            continue
        result = analyze.parse_run_to_result(str(log_path))
        result["run_id"] = log_path.stem
        result["log_path"] = str(log_path)
        result["seed"] = seed
        rows.append(result)
    if not rows:
        print("no successful runs collected", file=sys.stderr)
        return 1
    summary = _summarise(rows, args.label)
    _print_summary(summary)
    _write_outputs(Path(args.out_dir), args.label, summary, rows)
    return 0


def cmd_compare(args: argparse.Namespace) -> int:
    summaries: list[dict] = []
    for path_str in args.inputs:
        p = Path(path_str)
        if not p.is_file():
            print(f"missing input: {p}", file=sys.stderr)
            return 2
        summaries.append(json.loads(p.read_text(encoding="utf-8")))
    _print_compare(summaries)
    return 0


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------


def main() -> int:
    p = argparse.ArgumentParser(description="Monte Carlo / ablation harness for counter-UAS sim.")
    sub = p.add_subparsers(dest="mode", required=True)

    pa = sub.add_parser("aggregate", help="Compute MC summary from existing logs.")
    pa.add_argument("--logs-dir", default=str(WORKSPACE / "runs" / "logs"))
    pa.add_argument("--pattern", default="*.log")
    pa.add_argument("--label", default="aggregate")
    pa.add_argument("--out-dir", default=str(DEFAULT_OUT))
    pa.set_defaults(func=cmd_aggregate)

    pr = sub.add_parser("run", help="Run N sims via run_capture.py and aggregate.")
    pr.add_argument("--n", type=int, default=10)
    pr.add_argument("--seed-base", type=int, default=1)
    pr.add_argument("--scenario", choices=["single", "multi"], default="single")
    pr.add_argument("--timeout-s", type=float, default=14.0)
    pr.add_argument("--launch-args", type=str, default=None)
    pr.add_argument("--label", default="run")
    pr.add_argument("--out-dir", default=str(DEFAULT_OUT))
    pr.set_defaults(func=cmd_run)

    pc = sub.add_parser("compare", help="Print a side-by-side table from MC summary JSONs.")
    pc.add_argument("--inputs", nargs="+", required=True)
    pc.set_defaults(func=cmd_compare)

    args = p.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
