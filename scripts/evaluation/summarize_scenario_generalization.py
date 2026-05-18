#!/usr/bin/env python3
"""Summarize SG1 scenario-generalization MC JSONs into per-cell and worst-cell tables."""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import stats_helpers as stats  # noqa: E402


def _load_json(path: Path) -> dict | None:
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _fmt_float(value: float | None, digits: int = 3) -> str:
    if value is None or not math.isfinite(float(value)):
        return ""
    return f"{float(value):.{digits}f}"


def _ci_bound(ci: object, key: str) -> float | None:
    if isinstance(ci, dict) and key in ci:
        try:
            return float(ci[key])
        except (TypeError, ValueError):
            return None
    return None


def _maybe_make_failure_hist(csv_path: Path, hist_path: Path, *, enabled: bool) -> None:
    if not enabled or not csv_path.is_file():
        return
    cmd = [
        sys.executable,
        "scripts/evaluation/summarize_failure_classes.py",
        str(csv_path),
        "--out-json",
        str(hist_path),
    ]
    subprocess.run(cmd, check=True)


def _summary_row(
    *,
    arm: str,
    cell_tag: str,
    label: str,
    out_dir: Path,
) -> dict[str, str]:
    json_path = out_dir / f"{label}.json"
    csv_path = out_dir / f"{label}.csv"
    hist_path = out_dir / f"{label}_failure_hist.json"
    data = _load_json(json_path)
    if not data:
        return {
            "cell_tag": cell_tag,
            "arm": arm,
            "label": label,
            "status": "missing",
            "n_runs": "",
            "success_rate_pct": "",
            "success_rate_ci95_low_pct": "",
            "success_rate_ci95_high_pct": "",
            "cohort_tier": "",
            "miss_p95_m": "",
            "miss_p95_ci95_low_m": "",
            "miss_p95_ci95_high_m": "",
            "miss_mean_m": "",
            "miss_median_m": "",
            "intercept_time_p95_s": "",
            "intercept_time_p95_ci95_low_s": "",
            "intercept_time_p95_ci95_high_s": "",
            "intercept_time_mean_s": "",
            "json_path": str(json_path.resolve()),
            "csv_path": str(csv_path.resolve()),
            "failure_hist_path": str(hist_path.resolve()),
        }
    miss = data.get("miss_distance_m", {})
    tint = data.get("intercept_time_s", {})
    sr_ci = data.get("success_rate_ci95", {})
    miss_p95_ci = miss.get("p95_ci95", {})
    tint_p95_ci = tint.get("p95_ci95", {})
    n_runs = int(data.get("n_runs") or 0)
    return {
        "cell_tag": cell_tag,
        "arm": arm,
        "label": label,
        "status": "ok",
        "n_runs": str(n_runs),
        "success_rate_pct": _fmt_float(100.0 * float(data.get("success_rate", float("nan"))), 1),
        "success_rate_ci95_low_pct": _fmt_float(100.0 * _ci_bound(sr_ci, "lower"), 1) if _ci_bound(sr_ci, "lower") is not None else "",
        "success_rate_ci95_high_pct": _fmt_float(100.0 * _ci_bound(sr_ci, "upper"), 1) if _ci_bound(sr_ci, "upper") is not None else "",
        "cohort_tier": str(data.get("cohort_tier") or stats.cohort_tier(n_runs)),
        "miss_p95_m": _fmt_float(miss.get("p95")),
        "miss_p95_ci95_low_m": _fmt_float(_ci_bound(miss_p95_ci, "lower")),
        "miss_p95_ci95_high_m": _fmt_float(_ci_bound(miss_p95_ci, "upper")),
        "miss_mean_m": _fmt_float(miss.get("mean")),
        "miss_median_m": _fmt_float(miss.get("median")),
        "intercept_time_p95_s": _fmt_float(tint.get("p95")),
        "intercept_time_p95_ci95_low_s": _fmt_float(_ci_bound(tint_p95_ci, "lower")),
        "intercept_time_p95_ci95_high_s": _fmt_float(_ci_bound(tint_p95_ci, "upper")),
        "intercept_time_mean_s": _fmt_float(tint.get("mean")),
        "json_path": str(json_path.resolve()),
        "csv_path": str(csv_path.resolve()),
        "failure_hist_path": str(hist_path.resolve()),
    }


def _worst(rows: list[dict[str, str]], arm: str) -> dict[str, str]:
    ok = [r for r in rows if r["arm"] == arm and r["status"] == "ok"]
    if not ok:
        return {"arm": arm, "worst_sr_cell": "", "worst_sr_pct": "", "worst_sr_ci95_low_pct": "", "worst_miss_cell": "", "worst_miss_p95_m": "", "worst_miss_p95_ci95_high_m": "", "worst_tint_cell": "", "worst_tint_p95_s": "", "worst_tint_p95_ci95_high_s": ""}
    sr_cell = min(ok, key=lambda r: float(r["success_rate_pct"]))
    miss_cell = max(ok, key=lambda r: float(r["miss_p95_m"]))
    tint_cell = max(ok, key=lambda r: float(r["intercept_time_p95_s"]))
    return {
        "arm": arm,
        "worst_sr_cell": sr_cell["cell_tag"],
        "worst_sr_pct": sr_cell["success_rate_pct"],
        "worst_sr_ci95_low_pct": sr_cell.get("success_rate_ci95_low_pct", ""),
        "worst_miss_cell": miss_cell["cell_tag"],
        "worst_miss_p95_m": miss_cell["miss_p95_m"],
        "worst_miss_p95_ci95_high_m": miss_cell.get("miss_p95_ci95_high_m", ""),
        "worst_tint_cell": tint_cell["cell_tag"],
        "worst_tint_p95_s": tint_cell["intercept_time_p95_s"],
        "worst_tint_p95_ci95_high_s": tint_cell.get("intercept_time_p95_ci95_high_s", ""),
    }


def _write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--matrix-csv", type=Path, default=Path("scripts/evaluation/fixtures/scenario_generalization_a_vs_d_matrix.csv"))
    ap.add_argument("--out-dir", type=Path, default=Path("runs/mc"))
    ap.add_argument("--study", default="gnc_sg1")
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--seed-base", type=int, default=9701)
    ap.add_argument("--out-csv", type=Path, default=Path("runs/evaluation/gnc_sg1_a_vs_d_summary.csv"))
    ap.add_argument("--worst-csv", type=Path, default=Path("runs/evaluation/gnc_sg1_a_vs_d_worst_cells.csv"))
    ap.add_argument(
        "--make-failure-hists",
        action="store_true",
        help="Create runs/mc/<label>_failure_hist.json for every existing per-cell CSV.",
    )
    args = ap.parse_args()

    with args.matrix_csv.open(encoding="utf-8", newline="") as f:
        cells = [r["cell_tag"] for r in csv.DictReader(f) if r.get("cell_tag")]

    rows: list[dict[str, str]] = []
    for cell_tag in cells:
        for arm in ("A", "D"):
            label = f"{args.study}_{arm}_{cell_tag}_n{args.n}_s{args.seed_base}"
            csv_path = args.out_dir / f"{label}.csv"
            hist_path = args.out_dir / f"{label}_failure_hist.json"
            _maybe_make_failure_hist(csv_path, hist_path, enabled=args.make_failure_hists)
            rows.append(_summary_row(arm=arm, cell_tag=cell_tag, label=label, out_dir=args.out_dir))

    _write_csv(args.out_csv, rows)
    worst_rows = [_worst(rows, "A"), _worst(rows, "D")]
    _write_csv(args.worst_csv, worst_rows)

    print(f"Wrote {args.out_csv.resolve()}")
    print(f"Wrote {args.worst_csv.resolve()}")
    for row in worst_rows:
        if not row["worst_sr_cell"]:
            print(f"{row['arm']}: no completed cells found")
            continue
        print(
            f"{row['arm']}: worst_sr={row['worst_sr_pct']}% @ {row['worst_sr_cell']}; "
            f"worst_miss_p95={row['worst_miss_p95_m']} m @ {row['worst_miss_cell']}; "
            f"worst_tint_p95={row['worst_tint_p95_s']} s @ {row['worst_tint_cell']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
