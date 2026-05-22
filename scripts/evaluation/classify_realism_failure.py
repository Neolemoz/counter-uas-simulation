#!/usr/bin/env python3
"""Additive realism-stressor failure taxonomy.

This helper is intentionally secondary to the existing F1–F5 failure taxonomy. It uses
additive realism metrics to label realism-shaped degradation modes without changing the
stable parser-visible success/miss/intercept-time contracts or replacing the primary
failure buckets.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict as dc_asdict
from pathlib import Path

_SCRIPTS_DIR = Path(__file__).resolve().parents[1]
_EVAL_DIR = Path(__file__).resolve().parent
for p in (_SCRIPTS_DIR, _EVAL_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import summarize_run  # noqa: E402
from realism_metrics import realism_metrics_summary  # noqa: E402
from selection_audit import selection_audit_summary  # noqa: E402


def classify_realism_failure(row: dict) -> str:
    if bool(row.get('hit', False)):
        return 'R0_none'

    stale = int(row.get('stale_detection_count', 0) or 0)
    delayed = int(row.get('delayed_detection_count', 0) or 0)
    bursts = int(row.get('dropout_burst_count', 0) or 0)
    coast = int(row.get('track_coast_count', 0) or 0)
    recover = int(row.get('track_recovery_count', 0) or 0)
    max_missed = int(row.get('max_track_missed_frames', 0) or 0)

    if stale > 0 and coast > 0 and recover == 0:
        return 'R1_stale_induced'
    if delayed > 0 and coast > 0:
        return 'R2_delayed_association'
    if bursts > 0 and max_missed >= 3 and recover == 0:
        return 'R3_coast_collapse'
    if max_missed >= 2 and coast > recover:
        return 'R4_timing_divergence'
    if stale > 0 or delayed > 0 or bursts > 0:
        return 'R5_realism_stress_other'
    return 'R0_none'


def main() -> int:
    ap = argparse.ArgumentParser(description='Print additive realism_failure_class JSON for one run log.')
    ap.add_argument('log', type=Path)
    ap.add_argument('--meta', type=Path, default=None)
    args = ap.parse_args()
    if not args.log.is_file():
        print(f'missing log: {args.log}', file=sys.stderr)
        return 2
    text = args.log.read_text(encoding='utf-8', errors='replace')
    row = dc_asdict(summarize_run.parse_log(text, run_id=args.log.stem))
    row.update(selection_audit_summary(text))
    row.update(realism_metrics_summary(text))
    print(json.dumps({'realism_failure_class': classify_realism_failure(row), 'log': str(args.log)}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
