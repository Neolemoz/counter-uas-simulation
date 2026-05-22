#!/usr/bin/env python3
"""Additive selection/oracle divergence taxonomy for replay evaluation rows.

This classifier is evidence-only. It does not replace hit/miss summaries,
ambiguity failure classes, realism failure classes, or any runtime authority
surface.
"""

from __future__ import annotations

import argparse
import json
import math
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
from selection_audit import observer_visibility_summary  # noqa: E402
from selection_audit import selection_audit_summary  # noqa: E402


def _int(row: dict, key: str) -> int:
    try:
        return int(row.get(key, 0) or 0)
    except (TypeError, ValueError):
        return 0


def _float_or_none(value: object) -> float | None:
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _bool_or_none(value: object) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered == 'true':
            return True
        if lowered == 'false':
            return False
    return None


def classify_selection_oracle_divergence(row: dict) -> str:
    """Return additive D0-D5 selection/oracle divergence class."""

    n_selection_blocks = _int(row, 'n_selection_blocks')
    oracle_match_rate = _float_or_none(row.get('selection_oracle_match_rate'))
    last_oracle_match = _bool_or_none(row.get('last_oracle_match'))
    observer_windows = _int(row, 'observer_visibility_windows')
    tracks_state_obs = _int(row, 'tracks_state_observation_count')

    if n_selection_blocks <= 0 or oracle_match_rate is None:
        return 'D5_inconclusive_visibility_limited'
    if observer_windows <= 0 and tracks_state_obs <= 0:
        return 'D5_inconclusive_visibility_limited'

    mismatch = oracle_match_rate < 0.95 or last_oracle_match is False
    if not mismatch:
        return 'D0_clean_aligned_selection'

    track_deleted = _int(row, 'track_deleted_count')
    unrecovered_coast = _int(row, 'unrecovered_coast_track_count')
    track_coast = _int(row, 'track_coast_count')
    track_recovery = _int(row, 'track_recovery_count')
    candidate_discard = _int(row, 'candidate_discard_count')
    continuity_changes = _int(row, 'track_continuity_change_count')
    recovery_thrash = _int(row, 'recovery_thrash_count')
    selection_proxy_events = _int(row, 'selection_proxy_event_count')
    ambiguity_class = str(row.get('ambiguity_failure_class') or '')
    realism_class = str(row.get('realism_failure_class') or '')

    deletion_pressure = (
        track_deleted >= 20
        or unrecovered_coast >= 25
        or (track_coast >= 400 and track_recovery == 0)
    )
    churn_pressure = (
        candidate_discard >= 10
        or continuity_changes >= 10
        or recovery_thrash >= 20
        or selection_proxy_events >= 8
        or ambiguity_class == 'A3_merge_instability'
        or realism_class == 'R4_timing_divergence'
    )

    if deletion_pressure and not churn_pressure:
        return 'D4_deletion_dominated_collapse'
    if churn_pressure:
        return 'D3_churn_driven_instability'
    if oracle_match_rate <= 0.60 and n_selection_blocks >= 4:
        return 'D2_persistent_oracle_mismatch'
    return 'D1_late_stage_divergence'


def main() -> int:
    ap = argparse.ArgumentParser(description='Print additive selection/oracle divergence class JSON for one run log.')
    ap.add_argument('log', type=Path)
    args = ap.parse_args()
    if not args.log.is_file():
        print(f'missing log: {args.log}', file=sys.stderr)
        return 2
    text = args.log.read_text(encoding='utf-8', errors='replace')
    row = dc_asdict(summarize_run.parse_log(text, run_id=args.log.stem))
    row.update(selection_audit_summary(text))
    row.update(observer_visibility_summary(text))
    row.update(realism_metrics_summary(text))
    print(
        json.dumps(
            {
                'selection_oracle_divergence_class': classify_selection_oracle_divergence(row),
                'log': str(args.log),
            },
            sort_keys=True,
        ),
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
