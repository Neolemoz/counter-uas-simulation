#!/usr/bin/env python3
"""
Validate heatmap P(hit) vs Gazebo interception outcomes using run_capture + parse_run_to_result.

Requires: colcon build, heatmap CSV (e.g. runs/intercept_heatmap_export/intercept_heatmap_prob_latest.csv).
Target start positions are passed via gazebo_target.launch.py launch args (target_start_*_m).
LOS / closing behavior stays default in launch (attack_los_to_origin + los_closing_speed_m_s).

By default runs **headless** (``use_gazebo_gui:=false``) so batch validation does not open the Gazebo GUI.
"""

from __future__ import annotations

import argparse
import csv
import random
import sys
import time
from pathlib import Path
from typing import TypedDict

_SCRIPTS = Path(__file__).resolve().parent
_REPO = _SCRIPTS.parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from analyze_run import parse_run_to_result  # noqa: E402
from run_capture import run_capture  # noqa: E402

P_COL_CANDIDATES = ("p_hit_noise_model", "p_hit", "p")


class ValidationCell(TypedDict):
    x: float
    y: float
    z: float
    p_hit: float
    tier: str  # "high" | "mid" | "low" | "pad"


def _prob_col(fieldnames: list[str] | None) -> str:
    if not fieldnames:
        return "p_hit_noise_model"
    low = {f.lower(): f for f in fieldnames}
    for key in P_COL_CANDIDATES:
        if key in low:
            return low[key]
        if key in fieldnames:
            return key
    return fieldnames[-1]


def _row_to_cell(row: tuple[float, float, float, float], tier: str) -> ValidationCell:
    return {
        "x": float(row[0]),
        "y": float(row[1]),
        "z": float(row[2]),
        "p_hit": float(row[3]),
        "tier": tier,
    }


def select_validation_cells(heatmap_csv_path: str, *, rng_seed: int = 42) -> list[ValidationCell]:
    """
    Load heatmap CSV; prefer 3 high (P>0.8), 3 mid [0.4,0.8], 3 low (P<0.4), each distinct by position.
    If fewer than 9, append ``tier=pad`` cells from P-quantiles (see warning on stdout).
    """
    path = Path(heatmap_csv_path)
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        pcol = _prob_col(reader.fieldnames)
        rows: list[tuple[float, float, float, float]] = []
        for r in reader:
            try:
                x = float(r["x_m"])
                y = float(r["y_m"])
                z = float(r["z_m"])
                p = float(r[pcol])
            except (KeyError, ValueError, TypeError):
                continue
            rows.append((x, y, z, p))

    high = [t for t in rows if t[3] > 0.8]
    mid = [t for t in rows if 0.4 <= t[3] <= 0.8]
    low = [t for t in rows if t[3] < 0.4]

    rng = random.Random(rng_seed)

    def _pick_unique(
        candidates: list[tuple[float, float, float, float]],
        n: int,
    ) -> list[tuple[float, float, float, float]]:
        seen: set[tuple[float, float, float]] = set()
        shuffled = list(candidates)
        rng.shuffle(shuffled)
        out: list[tuple[float, float, float, float]] = []
        for row in shuffled:
            key = (round(row[0], 3), round(row[1], 3), round(row[2], 3))
            if key in seen:
                continue
            seen.add(key)
            out.append(row)
            if len(out) >= n:
                break
        return out

    sel: list[ValidationCell] = []
    for row in _pick_unique(high, 3):
        sel.append(_row_to_cell(row, "high"))
    for row in _pick_unique(mid, 3):
        sel.append(_row_to_cell(row, "mid"))
    for row in _pick_unique(low, 3):
        sel.append(_row_to_cell(row, "low"))

    taken = {(round(c["x"], 3), round(c["y"], 3), round(c["z"], 3)) for c in sel}
    if len(sel) < 9:
        print(
            f"[validate] warning: tier picks={len(sel)} "
            f"(pool high={len(high)} mid={len(mid)} low={len(low)}); "
            f"padding with P-quantiles (tier=pad)",
            flush=True,
        )
        all_sorted = sorted(rows, key=lambda t: t[3])
        if all_sorted:
            need = 9 - len(sel)
            for k in range(need):
                idx = int(round((k + 1) * (len(all_sorted) - 1) / max(1, need + 1)))
                idx = max(0, min(len(all_sorted) - 1, idx))
                cand = all_sorted[idx]
                key = (round(cand[0], 3), round(cand[1], 3), round(cand[2], 3))
                if key in taken:
                    continue
                taken.add(key)
                sel.append(_row_to_cell(cand, "pad"))
                if len(sel) >= 9:
                    break
            if len(sel) < 9:
                for cand in all_sorted:
                    key = (round(cand[0], 3), round(cand[1], 3), round(cand[2], 3))
                    if key in taken:
                        continue
                    taken.add(key)
                    sel.append(_row_to_cell(cand, "pad"))
                    if len(sel) >= 9:
                        break

    return sel[:9]


def _launch_args_for_cell(
    x: float,
    y: float,
    z: float,
    *,
    headless: bool,
) -> str:
    base = (
        f"target_start_x_m:={x:.6f} target_start_y_m:={y:.6f} target_start_z_m:={z:.6f}"
    )
    if headless:
        return f"{base} use_gazebo_gui:=false"
    return f"{base} use_gazebo_gui:=true"


def run_validation_trials(
    cells: list[ValidationCell],
    runs_per_cell: int,
    *,
    scenario: str = "single",
    timeout_s: float = 90.0,
    notes_prefix: str = "heatmap_validation",
    headless: bool = True,
) -> list[dict]:
    """
    For each cell, run Gazebo ``runs_per_cell`` times via ``run_capture``; parse logs with ``parse_run_to_result``.

    *headless*: if True, pass ``use_gazebo_gui:=false`` (gz server-only) to avoid GUI freezes / DISPLAY needs.
    """
    if runs_per_cell < 1:
        raise ValueError("runs_per_cell must be >= 1")
    results: list[dict] = []
    for cell in cells:
        cx, cy, cz, p_hm = cell["x"], cell["y"], cell["z"], cell["p_hit"]
        tier = cell["tier"]
        ok = 0
        total = int(runs_per_cell)
        for ri in range(total):
            note = f"{notes_prefix} cell=({cx:.3f},{cy:.3f},{cz:.3f}) run={ri + 1}/{total} p_heatmap={p_hm:.4f}"
            launch_args = _launch_args_for_cell(cx, cy, cz, headless=headless)
            log_path, _meta_path, _meta_obj, _rc = run_capture(
                scenario=scenario,
                timeout_s=float(timeout_s),
                notes=note,
                launch_args=launch_args,
            )
            try:
                pr = parse_run_to_result(str(log_path))
                if bool(pr.get("success")):
                    ok += 1
            except Exception as exc:
                print(f"[validate] parse failed {log_path}: {exc}", flush=True)
        results.append(
            {
                "cell": (float(cx), float(cy), float(cz)),
                "tier": tier,
                "p_heatmap": float(p_hm),
                "success_count": int(ok),
                "total_runs": int(total),
            },
        )
    return results


def compute_validation_metrics(
    trial_results: list[dict],
) -> tuple[list[dict], float, float]:
    """
    Build comparison rows with p_gazebo = success_count/total_runs and absolute error vs p_heatmap.
    Returns (table_rows, mean_error, max_error).
    """
    rows: list[dict] = []
    errors: list[float] = []
    for r in trial_results:
        cell = r["cell"]
        tier = r.get("tier", "")
        p_hm = float(r["p_heatmap"])
        tot = int(r["total_runs"])
        if tot < 1:
            raise ValueError("total_runs must be >= 1 in compute_validation_metrics")
        ok = int(r["success_count"])
        p_gz = float(ok) / float(tot)
        err = abs(p_gz - p_hm)
        errors.append(err)
        rows.append(
            {
                "cell": cell,
                "tier": tier,
                "p_heatmap": p_hm,
                "p_gazebo": p_gz,
                "error": err,
            },
        )
    mean_e = sum(errors) / len(errors) if errors else float("nan")
    max_e = max(errors) if errors else float("nan")
    return rows, float(mean_e), float(max_e)


def _verdict(mean_e: float, max_e: float) -> str:
    if not (mean_e == mean_e):  # nan
        return "POOR"
    if mean_e <= 0.15 and max_e <= 0.25:
        return "GOOD"
    if mean_e <= 0.30 and max_e <= 0.45:
        return "PARTIAL"
    return "POOR"


def _print_table(table: list[dict]) -> None:
    print("\n=== validation table (P_heatmap vs P_gazebo) ===", flush=True)
    for row in table:
        cx, cy, cz = row["cell"]
        tier = row.get("tier", "")
        print(
            f"  [{tier:4s}] cell=({cx:.1f},{cy:.1f},{cz:.1f})  "
            f"p_heatmap={row['p_heatmap']:.3f}  p_gazebo={row['p_gazebo']:.3f}  "
            f"|err|={row['error']:.3f}",
            flush=True,
        )
    print("", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--heatmap-csv",
        type=Path,
        default=_REPO / "runs" / "intercept_heatmap_export" / "intercept_heatmap_prob_latest.csv",
    )
    ap.add_argument("--runs-per-cell", type=int, default=2, help="Monte Carlo trials per validation cell (>=1)")
    ap.add_argument("--timeout-s", type=float, default=90.0)
    ap.add_argument("--scenario", choices=["single", "multi"], default="single")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--dry-run", action="store_true", help="only print selected cells; no Gazebo")
    ap.add_argument(
        "--gazebo-gui",
        action="store_true",
        help="Open Gazebo 3D window (default: headless via use_gazebo_gui:=false)",
    )
    ap.add_argument("--out-csv", type=Path, default=None, help="optional path to save summary CSV")
    args = ap.parse_args()

    if int(args.runs_per_cell) < 1:
        print("--runs-per-cell must be >= 1", file=sys.stderr)
        return 2

    cells = select_validation_cells(str(args.heatmap_csv), rng_seed=int(args.seed))
    if not cells:
        print("No cells selected (empty CSV or missing p column).", file=sys.stderr)
        return 2

    print(f"[validate] selected {len(cells)} cells from {args.heatmap_csv}", flush=True)
    for c in cells:
        print(
            f"  [{c['tier']:4s}] ({c['x']:.3f}, {c['y']:.3f}, {c['z']:.3f})  P={c['p_hit']:.4f}",
            flush=True,
        )

    if args.dry_run:
        return 0

    headless = not bool(args.gazebo_gui)
    if headless:
        print("[validate] headless Gazebo (use_gazebo_gui:=false)", flush=True)
    (_REPO / "runs" / "validation").mkdir(parents=True, exist_ok=True)

    trial_results = run_validation_trials(
        cells,
        int(args.runs_per_cell),
        scenario=str(args.scenario),
        timeout_s=float(args.timeout_s),
        headless=headless,
    )
    table, mean_e, max_e = compute_validation_metrics(trial_results)
    verdict = _verdict(mean_e, max_e)

    _print_table(table)
    print(f"mean |error|: {mean_e:.4f}", flush=True)
    print(f"max |error|:  {max_e:.4f}", flush=True)
    print(f"conclusion: {verdict}", flush=True)

    if args.out_csv is not None:
        args.out_csv.parent.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        outp = args.out_csv
        if outp.is_dir():
            outp = outp / f"heatmap_validation_{stamp}.csv"
        with outp.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(
                f,
                fieldnames=[
                    "tier",
                    "x_m",
                    "y_m",
                    "z_m",
                    "p_heatmap",
                    "p_gazebo",
                    "error",
                    "success_count",
                    "total_runs",
                ],
            )
            w.writeheader()
            for row, tr in zip(table, trial_results):
                cx, cy, cz = row["cell"]
                w.writerow(
                    {
                        "tier": row.get("tier", ""),
                        "x_m": cx,
                        "y_m": cy,
                        "z_m": cz,
                        "p_heatmap": row["p_heatmap"],
                        "p_gazebo": row["p_gazebo"],
                        "error": row["error"],
                        "success_count": tr["success_count"],
                        "total_runs": tr["total_runs"],
                    },
                )
        print(f"[validate] wrote {outp}", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
