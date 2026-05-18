#!/usr/bin/env python3
"""Histogram F1–F5 failure_class over logs listed in a monte_carlo per-run CSV (log_path column)."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
for parent in (_EVAL, _EVAL.parent):
    if str(parent) not in sys.path:
        sys.path.insert(0, str(parent))

import stats_helpers as stats  # noqa: E402
from classify_run import classify_run_failure_evidence  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description='Count failure_class over MC CSV log_path column.')
    ap.add_argument('csv_path', type=Path, help='monte_carlo *.csv with log_path header')
    ap.add_argument(
        '--out-json',
        type=Path,
        default=None,
        help='Optional path to write {"failure_hist": {...}, ...}',
    )
    args = ap.parse_args()
    p = args.csv_path
    if not p.is_file():
        print(f'missing CSV: {p}', file=sys.stderr)
        return 2
    with p.open(encoding='utf-8', newline='') as f:
        r = csv.DictReader(f)
        if 'log_path' not in (r.fieldnames or []):
            print('CSV must include log_path column', file=sys.stderr)
            return 2
        rows = list(r)

    hist: Counter[str] = Counter()
    cohorts: set[str] = set()
    evidence_rows: list[dict[str, object]] = []
    missing_logs: list[str] = []
    for row in rows:
        lp = (row.get('log_path') or '').strip()
        if not lp:
            continue
        log_path = Path(lp)
        if not log_path.is_file():
            missing_logs.append(lp)
            continue
        mp = log_path.with_suffix('.meta.json')
        if mp.is_file():
            try:
                md = json.loads(mp.read_text(encoding='utf-8'))
                co = md.get('cohort')
                if co is not None and str(co).strip():
                    cohorts.add(str(co).strip())
            except (OSError, json.JSONDecodeError):
                pass
        evidence = classify_run_failure_evidence(log_path, capture_rc=None)
        hist[str(evidence['failure_class'])] += 1
        evidence_rows.append(evidence)

    total = int(sum(hist.values()))
    class_ci95 = {
        name: stats.wilson_ci(count, total)
        for name, count in sorted(hist.items())
    }
    f5_count = int(hist.get('F5_unknown', 0))

    payload = {
        'csv': str(p.resolve()),
        'n_classified': total,
        'failure_hist': dict(sorted(hist.items())),
        'failure_class_ci95': class_ci95,
        'f5_unknown_rate': (f5_count / total) if total else None,
        'f5_unknown_ci95': stats.wilson_ci(f5_count, total),
        'evidence_rows': evidence_rows,
        'missing_logs': missing_logs,
        'meta_cohorts_seen': sorted(cohorts),
    }
    txt = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    print(txt, end='')
    if args.out_json:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        args.out_json.write_text(txt, encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
