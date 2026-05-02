#!/usr/bin/env python3
"""Batch multiple Gazebo capture runs; optional parse-all helper."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

# Resolve sibling `run_capture` when this file is run as a script.
_SCRIPTS_DIR = Path(__file__).resolve().parent
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

from analyze_run import parse_run_to_result
from run_capture import run_capture


def parse_all_runs(log_paths: list) -> list:
    """
    Parse each log path with ``parse_run_to_result``; return a list of dicts.

    Skips paths that cannot be read or parsed without changing
    ``parse_run_to_result`` / ``parse_log`` behavior.
    """
    results: list[dict] = []
    for log_path in log_paths:
        p = str(log_path)
        try:
            results.append(parse_run_to_result(p))
        except Exception as exc:
            print(f"[parse_all_runs] skip: {p!r}: {exc}", flush=True)
    return results


def aggregate_results(results: list) -> dict:
    """
    Aggregate per-run dicts (e.g. from ``parse_run_to_result``) into summary stats.

    ``intercept_time_s`` may be None; those entries are ignored for ``avg_time``
    (NaN-safe mean). ``miss_distance_m`` may be NaN; ``nanmean`` / ``nanstd`` apply.
    """
    if not results:
        nan = float("nan")
        return {
            "success_rate": nan,
            "avg_miss": nan,
            "std_miss": nan,
            "avg_time": nan,
        }

    n_total = int(len(results))
    n_ok = sum(1 for r in results if isinstance(r, dict) and r.get("success") is True)
    success_rate = float(n_ok) / float(n_total) if n_total > 0 else float("nan")

    miss_vals: list[float] = []
    time_vals: list[float] = []
    for r in results:
        if not isinstance(r, dict):
            miss_vals.append(np.nan)
            time_vals.append(np.nan)
            continue
        m = r.get("miss_distance_m")
        miss_vals.append(float(m) if m is not None and np.isfinite(float(m)) else np.nan)
        t = r.get("intercept_time_s")
        if t is None:
            time_vals.append(np.nan)
        else:
            tf = float(t)
            time_vals.append(tf if np.isfinite(tf) else np.nan)

    miss_arr = np.asarray(miss_vals, dtype=np.float64)
    time_arr = np.asarray(time_vals, dtype=np.float64)

    return {
        "success_rate": float(success_rate),
        "avg_miss": float(np.nanmean(miss_arr)),
        "std_miss": float(np.nanstd(miss_arr, ddof=0)),
        "avg_time": float(np.nanmean(time_arr)),
    }


def run_gazebo_batch(n_runs: int) -> list:
    """
    Run Gazebo via ``run_capture`` N times; return log file paths.

    Uses ``run_capture`` defaults: scenario ``single``, timeout 14 s,
    no notes, no extra launch args. Adjust here when you need batch
    randomization or different scenarios.
    """
    n = int(n_runs)
    log_paths: list[str] = []
    for i in range(n):
        print(f"[gazebo_batch] run {i + 1} / {n}", flush=True)
        log_path, _meta_path, _meta, _rc = run_capture(
            scenario="single",
            timeout_s=14.0,
            notes=None,
            launch_args=None,
        )
        log_paths.append(str(log_path))
    return log_paths


def _fmt_metric(v: float) -> str:
    if isinstance(v, float) and math.isnan(v):
        return "n/a"
    return f"{v:.4f}"


def run_gazebo_monte_carlo(n_runs: int) -> dict:
    """
    Full pipeline: batch capture → parse each log → aggregate → print summary.

    Prints ``success_rate``, ``avg_miss``, ``avg_time`` (from ``aggregate_results``).
    Returns the summary dict.
    """
    log_paths = run_gazebo_batch(n_runs)
    results = parse_all_runs(log_paths)
    summary = aggregate_results(results)

    print("\n=== Gazebo Monte Carlo summary ===", flush=True)
    print(f"success_rate: {_fmt_metric(summary['success_rate'])}", flush=True)
    print(f"avg_miss:     {_fmt_metric(summary['avg_miss'])} m", flush=True)
    print(f"avg_time:     {_fmt_metric(summary['avg_time'])} s", flush=True)
    print("===================================\n", flush=True)

    return summary


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(
        description="Gazebo Monte Carlo: N sequential captures, parse logs, print aggregate stats.",
    )
    p.add_argument("n_runs", type=int, help="Number of sequential runs.")
    args = p.parse_args()
    run_gazebo_monte_carlo(args.n_runs)
