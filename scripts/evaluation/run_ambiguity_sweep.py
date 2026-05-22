#!/usr/bin/env python3
"""Run a small matched-seed sensing-ambiguity sweep and emit compact aggregate CSV rows."""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FIXTURE = REPO / 'scripts' / 'evaluation' / 'fixtures' / 'ambiguity_sweep_profiles.csv'

_SCRIPTS_DIR = REPO / 'scripts'
_EVAL_DIR = REPO / 'scripts' / 'evaluation'
for p in (_SCRIPTS_DIR, _EVAL_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from evaluation_row import evaluation_row  # noqa: E402
from classify_ambiguity_failure import classify_ambiguity_failure  # noqa: E402
from classify_selection_oracle_divergence import classify_selection_oracle_divergence  # noqa: E402


def _run_cmd(cmd: list[str]) -> None:
    r = subprocess.run(cmd, cwd=REPO, text=True, capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr or r.stdout or f'command failed: {cmd}')
    if r.stdout:
        print(r.stdout, end='')


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _fmean(values: list[float]) -> float:
    return sum(values) / len(values) if values else float('nan')


def _safe_float(v: object) -> float | None:
    if v is None:
        return None
    try:
        out = float(v)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _summarize_eval_rows(rows: list[dict]) -> dict[str, str]:
    oracle_rates = [_safe_float(row.get('selection_oracle_match_rate')) for row in rows]
    oracle_rates_f = [v for v in oracle_rates if v is not None]

    def _sum_int(field: str) -> int:
        return sum(int(row.get(field, 0) or 0) for row in rows)

    ambiguity_classes: dict[str, int] = {}
    divergence_classes: dict[str, int] = {}
    for row in rows:
        cls = str(row.get('ambiguity_failure_class') or 'A0_none')
        ambiguity_classes[cls] = ambiguity_classes.get(cls, 0) + 1
        div_cls = str(row.get('selection_oracle_divergence_class') or 'D5_inconclusive_visibility_limited')
        divergence_classes[div_cls] = divergence_classes.get(div_cls, 0) + 1
    dominant_ambiguity = max(ambiguity_classes.items(), key=lambda kv: kv[1])[0] if ambiguity_classes else 'A0_none'
    dominant_divergence = (
        max(divergence_classes.items(), key=lambda kv: kv[1])[0]
        if divergence_classes
        else 'D5_inconclusive_visibility_limited'
    )

    return {
        'ghost_detection_count': str(_sum_int('ghost_detection_count')),
        'fragmented_gap_count': str(_sum_int('fragmented_gap_count')),
        'candidate_spawn_count': str(_sum_int('candidate_spawn_count')),
        'candidate_discard_count': str(_sum_int('candidate_discard_count')),
        'duplicate_track_merge_count': str(_sum_int('duplicate_track_merge_count')),
        'track_coast_count': str(_sum_int('track_coast_count')),
        'track_recovery_count': str(_sum_int('track_recovery_count')),
        'max_track_missed_frames': str(max((int(row.get('max_track_missed_frames', 0) or 0) for row in rows), default=0)),
        'selection_oracle_match_rate_mean': f'{_fmean(oracle_rates_f):.3f}',
        'n_selection_blocks': str(_sum_int('n_selection_blocks')),
        'selection_mismatch_count': str(_sum_int('selection_mismatch_count')),
        'mismatch_after_fragmented_gap_count': str(_sum_int('mismatch_after_fragmented_gap_count')),
        'dominant_ambiguity_failure_class': dominant_ambiguity,
        'dominant_selection_oracle_divergence_class': dominant_divergence,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profiles-csv', type=Path, default=FIXTURE)
    ap.add_argument('--default-scenario', choices=['single', 'multi'], default='single')
    ap.add_argument('--n', type=int, default=1)
    ap.add_argument('--seed-base', type=int, default=5101)
    ap.add_argument('--timeout-s', type=float, default=35.0)
    ap.add_argument('--study', default='ambiguity_realism')
    ap.add_argument('--out-dir', type=Path, default=REPO / 'runs' / 'mc')
    ap.add_argument('--out-csv', type=Path, default=REPO / 'runs' / 'evaluation' / 'ambiguity_sweep_summary.csv')
    ap.add_argument('--headless', action='store_true')
    args = ap.parse_args()

    with args.profiles_csv.open(encoding='utf-8', newline='') as f:
        profiles = list(csv.DictReader(f))

    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.out_csv.parent.mkdir(parents=True, exist_ok=True)

    rows_out: list[dict[str, str]] = []
    for prof in profiles:
        profile_id = str(prof.get('profile_id') or '').strip()
        if not profile_id:
            continue
        scenario = str(prof.get('scenario') or args.default_scenario).strip() or args.default_scenario
        cohort = f'{args.study}_{profile_id}'
        label = f'{args.study}_{profile_id}_n{args.n}_s{args.seed_base}'
        launch_args = str(prof.get('launch_args') or '').strip()
        if args.headless and 'use_gazebo_gui:=' not in launch_args:
            launch_args = f'{launch_args} use_gazebo_gui:=false'.strip()
        notes = str(prof.get('notes') or '').strip()
        cmd = [
            sys.executable,
            'scripts/monte_carlo.py',
            'run',
            '--n',
            str(args.n),
            '--seed-base',
            str(args.seed_base),
            '--scenario',
            scenario,
            '--timeout-s',
            str(args.timeout_s),
            '--label',
            label,
            '--out-dir',
            str(args.out_dir),
            '--cohort',
            cohort,
            '--launch-args',
            launch_args,
        ]
        print(f'[ambiguity_sweep] profile={profile_id} scenario={scenario} cohort={cohort}', flush=True)
        _run_cmd(cmd)
        summary = _load_json(args.out_dir / f'{label}.json')
        per_run_csv = args.out_dir / f'{label}.csv'
        per_rows: list[dict] = []
        with per_run_csv.open(encoding='utf-8', newline='') as f:
            for row in csv.DictReader(f):
                log_path = Path(str(row.get('log_path') or ''))
                meta_path = log_path.with_suffix('.meta.json')
                if log_path.is_file():
                    eval_row = evaluation_row(log_path, meta_path=meta_path if meta_path.is_file() else None)
                    eval_row['notes_meta'] = f"{eval_row.get('notes_meta') or ''} scenario={scenario} profile={profile_id} notes={notes}".strip()
                    eval_row['ambiguity_failure_class'] = classify_ambiguity_failure(eval_row)
                    eval_row['selection_oracle_divergence_class'] = classify_selection_oracle_divergence(eval_row)
                    per_rows.append(eval_row)
        derived = _summarize_eval_rows(per_rows)
        success_rate_pct = f"{100.0 * float(summary.get('success_rate', 0.0)):.1f}"
        miss_mean_m = f"{float(summary.get('miss_distance_m', {}).get('mean', float('nan'))):.3f}"
        intercept_time_mean_s = f"{float(summary.get('intercept_time_s', {}).get('mean', float('nan'))):.3f}"
        row = {
            'profile_id': profile_id,
            'scenario': scenario,
            'cohort': cohort,
            'label': label,
            'n_runs': str(summary.get('n_runs', '')),
            'success_rate_pct': success_rate_pct,
            'miss_mean_m': miss_mean_m,
            'intercept_time_mean_s': intercept_time_mean_s,
            'notes': notes,
            'launch_args': launch_args,
            'summary_json': str((args.out_dir / f'{label}.json').resolve()),
            'per_run_csv': str(per_run_csv.resolve()),
        }
        row.update(derived)
        rows_out.append(row)

    with args.out_csv.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()))
        w.writeheader()
        w.writerows(rows_out)
    print(f'[ambiguity_sweep] wrote {args.out_csv}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
