#!/usr/bin/env python3
"""Run a small matched-seed runtime-realism sweep and emit a compact comparison CSV.

The sweep is intentionally narrow: it reuses the existing Monte Carlo harness, keeps
seeds/cohorts explicit, and adds only realism-overlay launch args. Existing parser-visible
contracts remain unchanged.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FIXTURE = REPO / 'scripts' / 'evaluation' / 'fixtures' / 'runtime_realism_sweep_profiles.csv'


def _run_cmd(cmd: list[str]) -> None:
    r = subprocess.run(cmd, cwd=REPO, text=True, capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr or r.stdout or f'command failed: {cmd}')
    if r.stdout:
        print(r.stdout, end='')


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profiles-csv', type=Path, default=FIXTURE)
    ap.add_argument('--scenario', choices=['single', 'multi'], default='single')
    ap.add_argument('--n', type=int, default=2)
    ap.add_argument('--seed-base', type=int, default=4101)
    ap.add_argument('--timeout-s', type=float, default=40.0)
    ap.add_argument('--study', default='runtime_realism')
    ap.add_argument('--out-dir', type=Path, default=REPO / 'runs' / 'mc')
    ap.add_argument('--out-csv', type=Path, default=REPO / 'runs' / 'evaluation' / 'runtime_realism_sweep_summary.csv')
    ap.add_argument(
        '--headless',
        action='store_true',
        help='Append use_gazebo_gui:=false to every profile for server-only sweep runs.',
    )
    args = ap.parse_args()

    rows: list[dict[str, str]] = []
    with args.profiles_csv.open(encoding='utf-8', newline='') as f:
        profiles = list(csv.DictReader(f))

    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.out_csv.parent.mkdir(parents=True, exist_ok=True)

    for prof in profiles:
        profile_id = str(prof.get('profile_id') or '').strip()
        if not profile_id:
            continue
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
            str(args.scenario),
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
        print(f'[realism_sweep] profile={profile_id} cohort={cohort}', flush=True)
        _run_cmd(cmd)
        summary = _load_json(args.out_dir / f'{label}.json')
        per_run_csv = args.out_dir / f'{label}.csv'
        row = {
            'profile_id': profile_id,
            'cohort': cohort,
            'label': label,
            'n_runs': str(summary.get('n_runs', '')),
            'success_rate_pct': f"{100.0 * float(summary.get('success_rate', 0.0)):.1f}",
            'miss_mean_m': f"{float(summary.get('miss_distance_m', {}).get('mean', float('nan'))):.3f}",
            'miss_p95_m': f"{float(summary.get('miss_distance_m', {}).get('p95', float('nan'))):.3f}",
            'intercept_time_mean_s': f"{float(summary.get('intercept_time_s', {}).get('mean', float('nan'))):.3f}",
            'notes': notes,
            'launch_args': launch_args,
            'summary_json': str((args.out_dir / f'{label}.json').resolve()),
            'per_run_csv': str(per_run_csv.resolve()),
        }
        rows.append(row)

    with args.out_csv.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f'[realism_sweep] wrote {args.out_csv}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
