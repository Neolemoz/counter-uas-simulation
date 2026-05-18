#!/usr/bin/env python3
"""Layer C statistical validation reports for MC and paired cohorts."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
_SCRIPTS = _EVAL.parent
for parent in (_EVAL, _SCRIPTS):
    if str(parent) not in sys.path:
        sys.path.insert(0, str(parent))

import stats_helpers as stats  # noqa: E402
from classify_run import classify_run_failure  # noqa: E402


def _boolish(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def _float_or_nan(value: object) -> float:
    try:
        f = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return float("nan")
    return f if math.isfinite(f) else float("nan")


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as f:
        return [dict(r) for r in csv.DictReader(f)]


def _meta_for_row(row: dict[str, str]) -> dict:
    meta_path = (row.get("meta_path") or "").strip()
    if not meta_path and (row.get("log_path") or "").strip():
        meta_path = str(Path(row["log_path"]).with_suffix(".meta.json"))
    if not meta_path:
        return {}
    p = Path(meta_path)
    if not p.is_file():
        return {}
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _note_value(notes: str, key: str) -> str:
    quoted = re.search(rf'\b{re.escape(key)}="([^"]+)"', notes)
    if quoted:
        return quoted.group(1)
    bare = re.search(rf"\b{re.escape(key)}=([^\s]+)", notes)
    if bare:
        return bare.group(1)
    return ""


def seed_for_row(row: dict[str, str]) -> int | None:
    for key in ("noise_seed_mc", "seed", "noise_seed"):
        raw = (row.get(key) or "").strip()
        if raw:
            try:
                return int(float(raw))
            except ValueError:
                pass
    meta = _meta_for_row(row)
    notes = str(meta.get("notes") or row.get("notes") or "")
    raw = _note_value(notes, "seed")
    if raw:
        try:
            return int(raw)
        except ValueError:
            return None
    launch = meta.get("launch_args_kv") if isinstance(meta.get("launch_args_kv"), dict) else {}
    raw = str(launch.get("noise_seed") or "")
    if raw:
        try:
            return int(float(raw))
        except ValueError:
            return None
    return None


def geometry_for_row(row: dict[str, str]) -> str:
    if (row.get("geometry_id") or "").strip():
        return str(row["geometry_id"]).strip()
    meta = _meta_for_row(row)
    return _note_value(str(meta.get("notes") or row.get("notes") or ""), "geometry_id")


def _metric_values(rows: list[dict[str, str]], key: str) -> list[float]:
    return [v for v in (_float_or_nan(r.get(key)) for r in rows) if math.isfinite(v)]


def aggregate_report(
    rows: list[dict[str, str]],
    *,
    label: str,
    summary: dict | None = None,
    bootstrap_seed: int = 1,
) -> dict[str, object]:
    successes = sum(1 for r in rows if _boolish(r.get("success")))
    n = len(rows)
    miss = _metric_values(rows, "miss_distance_m")
    tint = _metric_values(rows, "intercept_time_s")
    cohorts = sorted({str(r.get("cohort") or "").strip() for r in rows if str(r.get("cohort") or "").strip()})
    seeds = [s for s in (seed_for_row(r) for r in rows) if s is not None]
    geometries = sorted({g for g in (geometry_for_row(r) for r in rows) if g})
    payload: dict[str, object] = {
        "label": label,
        "n_runs": n,
        "cohort_tier": stats.cohort_tier(n),
        "small_n_warning": ""
        if n >= 40
        else (
            "pilot cohort only; N>=40 is required for research-grade validation guidance"
            if n >= 10
            else "smoke runs are wiring checks only; do not promote statistical claims"
        ),
        "cohorts_seen": cohorts,
        "seeds": {
            "n_with_seed": len(seeds),
            "min": min(seeds) if seeds else None,
            "max": max(seeds) if seeds else None,
            "duplicates": sorted(k for k, v in Counter(seeds).items() if v > 1),
        },
        "geometry_ids": geometries,
        "success_rate": {
            "n": n,
            "successes": successes,
            "estimate": successes / n if n else float("nan"),
            "ci95": stats.wilson_ci(successes, n),
        },
        "miss_distance_m": {
            "n": len(miss),
            "p50": stats.percentile(miss, 50.0),
            "p50_ci95": stats.bootstrap_quantile_ci(miss, 50.0, seed=bootstrap_seed + 50),
            "p95": stats.percentile(miss, 95.0),
            "p95_ci95": stats.bootstrap_quantile_ci(miss, 95.0, seed=bootstrap_seed + 95),
        },
        "intercept_time_s": {
            "n": len(tint),
            "p50": stats.percentile(tint, 50.0),
            "p50_ci95": stats.bootstrap_quantile_ci(tint, 50.0, seed=bootstrap_seed + 150),
            "p95": stats.percentile(tint, 95.0),
            "p95_ci95": stats.bootstrap_quantile_ci(tint, 95.0, seed=bootstrap_seed + 195),
        },
        "source_summary": summary or {},
    }
    return payload


def _by_seed(rows: list[dict[str, str]]) -> tuple[dict[int, dict[str, str]], list[dict[str, str]]]:
    out: dict[int, dict[str, str]] = {}
    missing: list[dict[str, str]] = []
    for row in rows:
        seed = seed_for_row(row)
        if seed is None:
            missing.append(row)
            continue
        out[seed] = row
    return out, missing


def _failure_class(row: dict[str, str]) -> str:
    log_path = (row.get("log_path") or "").strip()
    if not log_path or not Path(log_path).is_file():
        return ""
    return classify_run_failure(Path(log_path), capture_rc=None)


def paired_report(
    baseline_rows: list[dict[str, str]],
    candidate_rows: list[dict[str, str]],
    *,
    bootstrap_seed: int = 1,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    base_by_seed, base_missing = _by_seed(baseline_rows)
    cand_by_seed, cand_missing = _by_seed(candidate_rows)
    seeds = sorted(set(base_by_seed) & set(cand_by_seed))
    rows_out: list[dict[str, object]] = []
    base_success: list[bool] = []
    cand_success: list[bool] = []
    miss_b: list[float] = []
    miss_c: list[float] = []
    tint_b: list[float] = []
    tint_c: list[float] = []
    transitions: Counter[str] = Counter()

    for seed in seeds:
        b = base_by_seed[seed]
        c = cand_by_seed[seed]
        bs = _boolish(b.get("success"))
        cs = _boolish(c.get("success"))
        mb = _float_or_nan(b.get("miss_distance_m"))
        mc = _float_or_nan(c.get("miss_distance_m"))
        tb = _float_or_nan(b.get("intercept_time_s"))
        tc = _float_or_nan(c.get("intercept_time_s"))
        fb = _failure_class(b)
        fc = _failure_class(c)
        if fb or fc:
            transitions[f"{fb or 'unclassified'}->{fc or 'unclassified'}"] += 1
        base_success.append(bs)
        cand_success.append(cs)
        if math.isfinite(mb) and math.isfinite(mc):
            miss_b.append(mb)
            miss_c.append(mc)
        if math.isfinite(tb) and math.isfinite(tc):
            tint_b.append(tb)
            tint_c.append(tc)
        rows_out.append(
            {
                "seed": seed,
                "geometry_id": geometry_for_row(b) or geometry_for_row(c),
                "base_ok": bs,
                "cand_ok": cs,
                "miss_b": mb,
                "miss_c": mc,
                "miss_delta_c_minus_b": mc - mb if math.isfinite(mb) and math.isfinite(mc) else "",
                "tint_b": tb,
                "tint_c": tc,
                "tint_delta_c_minus_b": tc - tb if math.isfinite(tb) and math.isfinite(tc) else "",
                "failure_b": fb,
                "failure_c": fc,
                "log_b": b.get("log_path", ""),
                "log_c": c.get("log_path", ""),
            }
        )

    report: dict[str, object] = {
        "matched_seed_count": len(seeds),
        "cohort_tier": stats.cohort_tier(len(seeds)),
        "baseline_seed_count": len(base_by_seed),
        "candidate_seed_count": len(cand_by_seed),
        "missing_seed_rows": {
            "baseline": len(base_missing),
            "candidate": len(cand_missing),
        },
        "baseline_only_seeds": sorted(set(base_by_seed) - set(cand_by_seed)),
        "candidate_only_seeds": sorted(set(cand_by_seed) - set(base_by_seed)),
        "paired_success": stats.paired_binary_summary(
            base_success,
            cand_success,
            seed=bootstrap_seed + 1,
        ),
        "miss_distance_m": {
            "paired_n": len(miss_b),
            "p50_delta_ci95": stats.paired_bootstrap_delta_ci(
                miss_b,
                miss_c,
                lambda xs: stats.percentile(xs, 50.0),
                statistic_name="p50",
                seed=bootstrap_seed + 50,
            ),
            "p95_delta_ci95": stats.paired_bootstrap_delta_ci(
                miss_b,
                miss_c,
                lambda xs: stats.percentile(xs, 95.0),
                statistic_name="p95",
                seed=bootstrap_seed + 95,
            ),
        },
        "intercept_time_s": {
            "paired_n": len(tint_b),
            "p95_delta_ci95": stats.paired_bootstrap_delta_ci(
                tint_b,
                tint_c,
                lambda xs: stats.percentile(xs, 95.0),
                statistic_name="p95",
                seed=bootstrap_seed + 195,
            ),
        },
        "failure_transitions": dict(sorted(transitions.items())),
        "caution": (
            ""
            if len(seeds) >= 40
            else "paired-seed comparison is below N>=40 validation guidance; treat as smoke/pilot evidence only"
        ),
    }
    return report, rows_out


def validate_manifest(manifest: dict, rows: list[dict[str, str]]) -> dict[str, object]:
    expected_n = int(manifest.get("n") or 0)
    expected_cohort = str(manifest.get("cohort") or "").strip()
    require_clean = bool(manifest.get("require_clean_git", False))
    seeds = [s for s in (seed_for_row(r) for r in rows) if s is not None]
    cohorts = sorted({str(r.get("cohort") or "").strip() for r in rows if str(r.get("cohort") or "").strip()})
    dirty_values = {str(r.get("git_dirty") or "").strip().lower() for r in rows if str(r.get("git_dirty") or "").strip()}
    missing_logs = [
        r.get("log_path", "")
        for r in rows
        if not (r.get("log_path") or "").strip() or not Path(str(r.get("log_path"))).is_file()
    ]
    missing_meta = [
        r.get("log_path", "")
        for r in rows
        if (r.get("log_path") or "").strip() and not Path(str(r.get("log_path"))).with_suffix(".meta.json").is_file()
    ]
    problems: list[str] = []
    if expected_n and len(rows) != expected_n:
        problems.append(f"row count {len(rows)} != manifest n {expected_n}")
    if expected_cohort and cohorts != [expected_cohort]:
        problems.append(f"cohorts seen {cohorts} do not match manifest cohort {expected_cohort!r}")
    duplicates = sorted(k for k, v in Counter(seeds).items() if v > 1)
    if duplicates:
        problems.append(f"duplicate seeds: {duplicates}")
    if len(seeds) != len(rows):
        problems.append(f"{len(rows) - len(seeds)} rows are missing seed metadata")
    if require_clean and dirty_values - {"false", "0"}:
        problems.append(f"dirty git rows present: {sorted(dirty_values)}")
    if missing_logs:
        problems.append(f"{len(missing_logs)} missing log paths")
    if missing_meta:
        problems.append(f"{len(missing_meta)} missing .meta.json sidecars")
    return {
        "ok": not problems,
        "problems": problems,
        "n_rows": len(rows),
        "cohorts_seen": cohorts,
        "seed_count": len(seeds),
        "duplicate_seeds": duplicates,
        "missing_logs": missing_logs,
        "missing_meta": missing_meta,
    }


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    ag = sub.add_parser("aggregate", help="Interval-aware summary for one MC CSV.")
    ag.add_argument("--input", type=Path, required=True, help="monte_carlo per-run CSV")
    ag.add_argument("--summary", type=Path, default=None, help="optional monte_carlo summary JSON")
    ag.add_argument("--label", default="", help="override label")
    ag.add_argument("--out-json", type=Path, required=True)
    ag.add_argument("--bootstrap-seed", type=int, default=1)

    pr = sub.add_parser("paired", help="Paired-seed comparison for two MC CSVs.")
    pr.add_argument("--baseline", type=Path, required=True)
    pr.add_argument("--candidate", type=Path, required=True)
    pr.add_argument("--out-json", type=Path, required=True)
    pr.add_argument("--out-csv", type=Path, default=None)
    pr.add_argument("--bootstrap-seed", type=int, default=1)

    vm = sub.add_parser("validate-manifest", help="Check one campaign manifest against an MC CSV.")
    vm.add_argument("--manifest", type=Path, required=True)
    vm.add_argument("--csv", type=Path, required=True)
    vm.add_argument("--out-json", type=Path, default=None)

    args = ap.parse_args()
    if args.cmd == "aggregate":
        rows = _read_csv(args.input)
        summary = None
        if args.summary and args.summary.is_file():
            summary = json.loads(args.summary.read_text(encoding="utf-8"))
        label = args.label or (summary or {}).get("label") or args.input.stem
        payload = aggregate_report(rows, label=str(label), summary=summary, bootstrap_seed=int(args.bootstrap_seed))
        _write_json(args.out_json, payload)
        print(f"Wrote {args.out_json.resolve()}")
        return 0
    if args.cmd == "paired":
        payload, rows_out = paired_report(
            _read_csv(args.baseline),
            _read_csv(args.candidate),
            bootstrap_seed=int(args.bootstrap_seed),
        )
        _write_json(args.out_json, payload)
        if args.out_csv:
            _write_csv(args.out_csv, rows_out)
        print(f"Wrote {args.out_json.resolve()}")
        if args.out_csv:
            print(f"Wrote {args.out_csv.resolve()}")
        return 0
    if args.cmd == "validate-manifest":
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        payload = validate_manifest(manifest, _read_csv(args.csv))
        text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
        print(text, end="")
        if args.out_json:
            _write_json(args.out_json, payload)
        return 0 if payload["ok"] else 1
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
