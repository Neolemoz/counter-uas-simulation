#!/usr/bin/env python3
"""
Threat / launch-parameter scenario matrix runner.

Reads a CSV (see scripts/evaluation/fixtures/threat_spawn_matrix_example.csv) and for each row
runs ``run_capture``, then merges ``evaluation_row`` aggregates into one output CSV.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path

_SCRIPTS_DIR = Path(__file__).resolve().parent
_EVAL_DIR = _SCRIPTS_DIR / "evaluation"
_REPO = _SCRIPTS_DIR.parent
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))
if str(_EVAL_DIR) not in sys.path:
    sys.path.insert(0, str(_EVAL_DIR))

from evaluation_row import evaluation_row  # noqa: E402
from classify_run import classify_run_failure  # noqa: E402
from run_capture import run_capture  # noqa: E402

KNOWN_FLOAT_KEYS = (
    "target_start_x_m",
    "target_start_y_m",
    "target_start_z_m",
    "target_los_closing_m_s",
    "target_los_dive_gain",
)


def row_to_launch_args(row: dict[str, str], *, headless: bool) -> str:
    fragments: list[str] = []
    for key in KNOWN_FLOAT_KEYS:
        val = str(row.get(key, "")).strip()
        if not val:
            continue
        try:
            f = float(val)
        except ValueError:
            sid = row.get("scenario_id")
            raise ValueError(f"non-numeric {key}={val!r} in scenario {sid!r}") from None
        fragments.append(f"{key}:={f:.6g}")
    lay = str(row.get("interceptor_ic_layout", "")).strip()
    if lay:
        fragments.append(f"interceptor_ic_layout:={lay}")
    extra = str(row.get("extra_launch_args", "")).strip()
    if extra:
        fragments.append(extra)
    if headless:
        fragments.append("use_gazebo_gui:=false")
    return " ".join(fragments)


def expand_row_output(row_flat: dict) -> dict:
    out = dict(row_flat)
    lor = out.get("last_oracle_ids")
    if isinstance(lor, list):
        out["last_oracle_ids"] = ";".join(lor)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Run scenario matrix CSV through Gazebo captures.")
    ap.add_argument(
        "--matrix-csv",
        type=Path,
        default=_REPO / "scripts/evaluation/fixtures/threat_spawn_matrix_example.csv",
    )
    ap.add_argument("--scenario", choices=["single", "multi"], default="single")
    ap.add_argument("--timeout-s", type=float, default=90.0)
    ap.add_argument("--out-csv", type=Path, default=_REPO / "runs" / "evaluation" / "scenario_matrix_latest.csv")
    ap.add_argument(
        "--gazebo-gui",
        action="store_true",
        help="If set, omit use_gazebo_gui:=false (opens Gazebo GUI).",
    )
    ap.add_argument(
        "--cohort",
        type=str,
        default="",
        help="Stored in capture .meta.json (same as run_capture --cohort). Overrides RUN_COHORT env.",
    )
    args = ap.parse_args()

    if not args.matrix_csv.is_file():
        print(f"matrix CSV missing: {args.matrix_csv}", file=sys.stderr)
        return 2

    args.out_csv.parent.mkdir(parents=True, exist_ok=True)
    aggregate_rows: list[dict[str, object]] = []
    cohort_eff = str(args.cohort or "").strip() or str(os.environ.get("RUN_COHORT", "") or "").strip()

    with args.matrix_csv.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for raw_table in reader:
            scenario_id = str(raw_table.get("scenario_id") or "").strip()
            if not scenario_id:
                continue
            headless = not bool(args.gazebo_gui)
            try:
                la = row_to_launch_args(raw_table, headless=headless)
            except ValueError as err:
                print(f"[matrix] skip {scenario_id}: {err}", file=sys.stderr)
                continue
            note = f"scenario_matrix id={scenario_id} {raw_table.get('notes_meta') or ''}"
            cohort_tag = cohort_eff or None
            log_path, meta_path_obj, _meta_obj, rc = run_capture(
                scenario=str(args.scenario),
                timeout_s=float(args.timeout_s),
                notes=note.strip(),
                launch_args=la,
                cohort=cohort_tag,
            )

            mp = Path(meta_path_obj) if isinstance(meta_path_obj, (str, Path)) else meta_path_obj
            summary = evaluation_row(Path(log_path), meta_path=mp if isinstance(mp, Path) else Path(str(mp)))
            merged: dict[str, object] = {**expand_row_output(summary)}
            merged["scenario_id"] = scenario_id
            merged["scenario_matrix_notes"] = raw_table.get("notes_meta") or ""
            merged["capture_rc"] = int(rc)
            try:
                merged["failure_class"] = classify_run_failure(Path(log_path), capture_rc=int(rc))
            except OSError:
                merged["failure_class"] = "F5_unknown"
            merged["launch_args"] = la
            aggregate_rows.append(merged)

    if not aggregate_rows:
        print("[matrix] no rows produced.", file=sys.stderr)
        return 1

    pref = [
        "scenario_id",
        "run_id",
        "hit",
        "min_miss_m",
        "layer_at_hit",
        "n_selection_blocks",
        "selection_oracle_match_rate",
        "last_selected",
        "last_oracle_ids",
        "last_oracle_match",
        "git_commit",
        "git_dirty",
        "capture_rc",
        "failure_class",
        "launch_args",
        "scenario_matrix_notes",
        "notes_meta",
    ]
    extra_keys: list[str] = []
    scan = set(pref)
    for r in aggregate_rows:
        for k in r.keys():
            if k not in scan:
                scan.add(k)
                extra_keys.append(k)
    fieldnames = pref + extra_keys

    with args.out_csv.open("w", newline="", encoding="utf-8") as outf:
        w = csv.DictWriter(outf, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for r in aggregate_rows:
            flat = {}
            for k in fieldnames:
                v = r.get(k)
                if isinstance(v, list):
                    v = ";".join(str(x) for x in v)
                flat[k] = v
            w.writerow(flat)

    print(f"[matrix] wrote {args.out_csv} rows={len(aggregate_rows)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
