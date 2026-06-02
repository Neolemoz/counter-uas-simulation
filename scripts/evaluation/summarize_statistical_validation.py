#!/usr/bin/env python3
"""Merge statistical validation sweep summaries, paired buckets, and failure histograms."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
_EVAL = Path(__file__).resolve().parent
_DEFAULT_PILOT_SUMMARY = REPO / 'runs' / 'evaluation' / 'statistical_validation_summary.csv'
_DEFAULT_FOCUSED_SUMMARY = REPO / 'runs' / 'evaluation' / 'statistical_validation_focused_n40_summary.csv'
_DEFAULT_OUT = REPO / 'runs' / 'evaluation' / 'statistical_validation_pilot_report.json'

_PAIRS = (
    ('predictive_baseline', 'predictive_intercept'),
    ('hysteresis_off', 'hysteresis_on'),
)

_GOVERNANCE_NOTICE = (
    'Descriptive aggregation only — not confidence intervals, operational effectiveness, '
    'or superiority claims.'
)

_SENSOR_TRIAGE = {
    'arm': 'sensor_realism_on',
    'pilot_n15_status': 'failed_zero_success',
    'diagnosis': {
        'timeout_issue': False,
        'target_geometry_issue': True,
        'intercept_source_issue': True,
        'overlay_config_issue': True,
        'notes': (
            'Pilot bringup logs show Active tracks: 0 for the full 90 s timeout with '
            'tracks_state intercept source. Default km-scale ingress without reachability '
            'geometry plus sensor overlay decimation/delay prevented track confirmation. '
            'Profile updated additively with wave5 reachability target_start; re-run deferred.'
        ),
    },
}

_MULTI_TRIAGE = {
    'arm': 'multi_defender',
    'pilot_n15_status': 'low_success_near_miss',
    'diagnosis': {
        'timeout_issue': False,
        'scenario_mismatch': False,
        'assignment_issue': False,
        'evaluation_parser_issue': True,
        'notes': (
            'Lab-scale multi world completes within timeout. min_miss ~1.6 m with '
            'hit_threshold 4.5 m but no [HIT] registration in logs — parse_run_to_result '
            'success=false despite proximity. End-state shows idle interceptors and '
            'targets at origin; kinetic HIT never emitted. Not an autonomy tuning issue.'
        ),
    },
}


def _load_summary_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding='utf-8', newline='') as f:
        return list(csv.DictReader(f))


def _row_by_label(rows: list[dict[str, str]]) -> dict[str, dict[str, str]]:
    return {str(r.get('label') or '').strip(): r for r in rows if str(r.get('label') or '').strip()}


def _failure_hist(csv_path: Path, out_json: Path) -> dict:
    cmd = [
        sys.executable,
        str(_EVAL / 'summarize_failure_classes.py'),
        str(csv_path),
        '--out-json',
        str(out_json),
    ]
    subprocess.run(cmd, cwd=REPO, check=True, capture_output=True, text=True)
    return json.loads(out_json.read_text(encoding='utf-8'))


def _pair_buckets(
    baseline_csv: Path,
    candidate_csv: Path,
    out_csv: Path,
    *,
    miss_delta: float,
    tint_delta: float,
) -> dict[str, object]:
    cmd = [
        sys.executable,
        str(_EVAL / 'pair_mc_seed_outcomes.py'),
        str(baseline_csv),
        str(candidate_csv),
        '--out-csv',
        str(out_csv),
        '--miss-delta',
        str(miss_delta),
        '--tint-delta',
        str(tint_delta),
    ]
    subprocess.run(cmd, cwd=REPO, check=True, capture_output=True, text=True)
    buckets: Counter[str] = Counter()
    paired = 0
    with out_csv.open(encoding='utf-8', newline='') as f:
        for row in csv.DictReader(f):
            paired += 1
            buckets[str(row.get('bucket') or '')] += 1
    return {
        'paired_seed_count': paired,
        'bucket_counts': dict(sorted(buckets.items())),
        'paired_csv': str(out_csv.resolve()),
    }


def _build_cohort(
    *,
    cohort_id: str,
    summary_csv: Path,
    out_dir: Path,
    miss_delta: float,
    tint_delta: float,
    pair_prefix: str,
) -> dict[str, object]:
    if not summary_csv.is_file():
        return {'cohort_id': cohort_id, 'status': 'missing_summary_csv', 'summary_csv': str(summary_csv)}

    rows = _load_summary_rows(summary_csv)
    out_dir.mkdir(parents=True, exist_ok=True)
    by_label = _row_by_label(rows)
    arms: list[dict[str, object]] = []
    for row in rows:
        label = str(row.get('label') or '').strip()
        per_run = Path(str(row.get('per_run_csv') or ''))
        hist_path = out_dir / f'{pair_prefix}_{label}_failure_hist.json'
        failure_hist: dict | None = None
        if per_run.is_file():
            failure_hist = _failure_hist(per_run, hist_path)
        arms.append(
            {
                'label': label,
                'scenario': row.get('scenario'),
                'n': row.get('n'),
                'seed_base': row.get('seed_base'),
                'success_rate_pct': row.get('success_rate_pct'),
                'miss_mean_m': row.get('miss_mean_m'),
                'miss_p95_m': row.get('miss_p95_m'),
                'intercept_time_mean_s': row.get('intercept_time_mean_s'),
                'summary_json': row.get('summary_json'),
                'failure_hist': failure_hist,
            }
        )

    comparisons: list[dict[str, object]] = []
    for base_label, cand_label in _PAIRS:
        base_row = by_label.get(base_label)
        cand_row = by_label.get(cand_label)
        if not base_row or not cand_row:
            comparisons.append(
                {
                    'baseline_label': base_label,
                    'candidate_label': cand_label,
                    'status': 'missing_arm',
                }
            )
            continue
        base_csv = Path(str(base_row.get('per_run_csv') or ''))
        cand_csv = Path(str(cand_row.get('per_run_csv') or ''))
        pair_csv = out_dir / f'{pair_prefix}_paired_{base_label}_vs_{cand_label}.csv'
        pair_summary = _pair_buckets(
            base_csv,
            cand_csv,
            pair_csv,
            miss_delta=miss_delta,
            tint_delta=tint_delta,
        )
        comparisons.append(
            {
                'baseline_label': base_label,
                'candidate_label': cand_label,
                'status': 'ok',
                'baseline_success_rate_pct': base_row.get('success_rate_pct'),
                'candidate_success_rate_pct': cand_row.get('success_rate_pct'),
                'baseline_miss_p95_m': base_row.get('miss_p95_m'),
                'candidate_miss_p95_m': cand_row.get('miss_p95_m'),
                **pair_summary,
            }
        )

    return {
        'cohort_id': cohort_id,
        'status': 'ok',
        'summary_csv': str(summary_csv.resolve()),
        'arms': arms,
        'paired_comparisons': comparisons,
    }


def build_report(
    *,
    pilot_summary_csv: Path,
    focused_summary_csv: Path | None,
    out_dir: Path,
    miss_delta: float,
    tint_delta: float,
) -> dict[str, object]:
    pilot = _build_cohort(
        cohort_id='pilot_n15_all_arms',
        summary_csv=pilot_summary_csv,
        out_dir=out_dir,
        miss_delta=miss_delta,
        tint_delta=tint_delta,
        pair_prefix='pilot_n15',
    )
    focused: dict[str, object] | None = None
    if focused_summary_csv is not None and focused_summary_csv.is_file():
        focused = _build_cohort(
            cohort_id='focused_n40_pairs',
            summary_csv=focused_summary_csv,
            out_dir=out_dir,
            miss_delta=miss_delta,
            tint_delta=tint_delta,
            pair_prefix='focused_n40',
        )

    return {
        'artifact_type': 'statistical_validation_pilot_report',
        'governance': {'notice': _GOVERNANCE_NOTICE},
        'pilot_n15': pilot,
        'focused_n40': focused,
        'triage': {
            'sensor_realism_on': _SENSOR_TRIAGE,
            'multi_defender': _MULTI_TRIAGE,
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pilot-summary-csv', type=Path, default=_DEFAULT_PILOT_SUMMARY)
    ap.add_argument('--focused-summary-csv', type=Path, default=_DEFAULT_FOCUSED_SUMMARY)
    ap.add_argument('--out-dir', type=Path, default=REPO / 'runs' / 'evaluation')
    ap.add_argument('--out-json', type=Path, default=_DEFAULT_OUT)
    ap.add_argument('--miss-delta', type=float, default=0.02)
    ap.add_argument('--tint-delta', type=float, default=0.05)
    ap.add_argument(
        '--skip-focused',
        action='store_true',
        help='Omit focused N=40 cohort when summary CSV is not yet available.',
    )
    args = ap.parse_args()

    if not args.pilot_summary_csv.is_file():
        print(f'missing pilot summary CSV: {args.pilot_summary_csv}', file=sys.stderr)
        return 2

    focused_path = None if args.skip_focused else args.focused_summary_csv
    args.out_dir.mkdir(parents=True, exist_ok=True)
    report = build_report(
        pilot_summary_csv=args.pilot_summary_csv,
        focused_summary_csv=focused_path,
        out_dir=args.out_dir,
        miss_delta=args.miss_delta,
        tint_delta=args.tint_delta,
    )
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(f'[summarize_statistical_validation] wrote {args.out_json}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
