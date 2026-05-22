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
for parent in (_EVAL.parent,):
    if str(parent) not in sys.path:
        sys.path.insert(0, str(parent))

from classify_run import classify_run_failure  # noqa: E402


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
    for row in rows:
        lp = (row.get('log_path') or '').strip()
        if not lp:
            continue
        log_path = Path(lp)
        mp = log_path.with_suffix('.meta.json')
        if mp.is_file():
            try:
                md = json.loads(mp.read_text(encoding='utf-8'))
                co = md.get('cohort')
                if co is not None and str(co).strip():
                    cohorts.add(str(co).strip())
            except (OSError, json.JSONDecodeError):
                pass
        hist[classify_run_failure(log_path, capture_rc=None)] += 1

    payload = {
        'csv': str(p.resolve()),
        'n_classified': int(sum(hist.values())),
        'failure_hist': dict(sorted(hist.items())),
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
