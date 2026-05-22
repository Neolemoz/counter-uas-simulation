#!/usr/bin/env python3
"""Offline failure bucket for evaluation logs — see roadmap Phase 1."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

_SCRIPTS_DIR = Path(__file__).resolve().parents[1]
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import summarize_run  # noqa: E402

_ENG_DELTA_RE = re.compile(r'delta_t_go_raw=([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)')
_REASSIGN_RE = re.compile(r'reassign|assignment_switch|switch_tti', re.IGNORECASE)


def classify_run_failure(
    log_path: Path,
    *,
    capture_rc: int | None = None,
) -> str:
    """
    Return F1..F5 bucket for a single Gazebo capture log.

    F1_timeout — run cut by timeout or obvious time limit.
    F2_geom_not_dyn — no HIT but geometry looked feasible in metrics.
    F3_track_instability — large jump in logged t_go (when [ENG_METRIC] present).
    F4_assignment — multi-assignment / switch hints in log.
    F5_unknown — default.
    """
    text = log_path.read_text(encoding='utf-8', errors='replace')
    low = text.lower()
    run_id = log_path.stem
    summary = summarize_run.parse_log(text, run_id=run_id)

    if '=== timeout ===' in low or (capture_rc is not None and int(capture_rc) == 124):
        return 'F1_timeout'

    if _REASSIGN_RE.search(text):
        return 'F4_assignment'

    deltas: list[float] = []
    for m in _ENG_DELTA_RE.finditer(text):
        try:
            deltas.append(float(m.group(1)))
        except ValueError:
            continue
    if deltas and (max(abs(d) for d in deltas) > 8.0 or len(deltas) > 15):
        return 'F3_track_instability'

    if not summary.hit and ('feasible_geom=true' in text or '[feas_warn]' in low):
        return 'F2_geom_not_dyn'

    return 'F5_unknown'


def main() -> int:
    ap = argparse.ArgumentParser(description='Print failure_class JSON for one run log.')
    ap.add_argument('log', type=Path)
    ap.add_argument('--capture-rc', type=int, default=None)
    ap.add_argument('--meta', type=Path, default=None, help='Optional .meta.json (for future use)')
    args = ap.parse_args()
    if not args.log.is_file():
        print(f'missing log: {args.log}', file=sys.stderr)
        return 2
    if args.meta is not None and args.meta.is_file():
        _ = json.loads(args.meta.read_text(encoding='utf-8'))
    bucket = classify_run_failure(args.log, capture_rc=args.capture_rc)
    print(json.dumps({'failure_class': bucket, 'log': str(args.log)}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
