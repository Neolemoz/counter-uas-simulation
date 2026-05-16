#!/usr/bin/env python3
"""Additive ambiguity-stressor taxonomy layered on top of stable evaluation summaries."""

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


def classify_ambiguity_failure(row: dict) -> str:
    if bool(row.get('hit', False)):
        return 'A0_none'

    ghost = int(row.get('ghost_detection_count', 0) or 0)
    frag = int(row.get('fragmented_gap_count', 0) or 0)
    candidate_spawn = int(row.get('candidate_spawn_count', 0) or 0)
    candidate_discard = int(row.get('candidate_discard_count', 0) or 0)
    merges = int(row.get('duplicate_track_merge_count', 0) or 0)
    coast = int(row.get('track_coast_count', 0) or 0)
    recover = int(row.get('track_recovery_count', 0) or 0)
    max_missed = int(row.get('max_track_missed_frames', 0) or 0)
    n_selection_blocks = int(row.get('n_selection_blocks', 0) or 0)
    oracle_match_rate = row.get('selection_oracle_match_rate')
    launch_notes = str(row.get('notes_meta') or '')
    crossing_hint = 'crossing' in launch_notes.lower() or 'scenario=multi' in launch_notes.lower()

    if crossing_hint and n_selection_blocks > 0 and oracle_match_rate is not None and float(oracle_match_rate) < 1.0:
        return 'A4_crossing_association_instability'
    if ghost > 0 and candidate_discard > 0 and candidate_spawn >= candidate_discard:
        return 'A1_ghost_induced_churn'
    if frag > 0 and coast > 0 and recover == 0 and max_missed >= 2:
        return 'A2_fragmentation_collapse'
    if merges > 0 and (ghost > 0 or frag > 0):
        return 'A3_merge_instability'
    if ghost > 0 or frag > 0 or merges > 0:
        return 'A5_ambiguity_stress_other'
    return 'A0_none'


def main() -> int:
    ap = argparse.ArgumentParser(description='Print additive ambiguity_failure_class JSON for one run log.')
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
    if args.meta and args.meta.is_file():
        try:
            meta = json.loads(args.meta.read_text(encoding='utf-8'))
        except (json.JSONDecodeError, OSError):
            meta = {}
        row['notes_meta'] = meta.get('notes')
    print(json.dumps({'ambiguity_failure_class': classify_ambiguity_failure(row), 'log': str(args.log)}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
