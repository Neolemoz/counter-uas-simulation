#!/usr/bin/env python3
"""Run matched-seed statistical validation MC profiles and emit a compact comparison CSV.

Reuses ``scripts/monte_carlo.py`` without changing parser-visible contracts. Profiles are
additive launch-arg arms for predictive intercept, hysteresis, multi-defender, aero limits,
and sensor realism overlay (bringup).
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FIXTURE = REPO / 'scripts' / 'evaluation' / 'fixtures' / 'statistical_validation_profiles.csv'
SUMMARY_FIELDS = (
    'label',
    'scenario',
    'n',
    'seed_base',
    'cohort',
    'mc_label',
    'n_runs',
    'success_rate_pct',
    'miss_mean_m',
    'miss_p95_m',
    'intercept_time_mean_s',
    'launch_args',
    'summary_json',
    'per_run_csv',
)


def _run_cmd(cmd: list[str], *, dry_run: bool) -> None:
    if dry_run:
        print('  ', ' '.join(cmd), flush=True)
        return
    r = subprocess.run(cmd, cwd=REPO, text=True, capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr or r.stdout or f'command failed: {cmd}')
    if r.stdout:
        print(r.stdout, end='')


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _int_field(row: dict[str, str], key: str, default: int) -> int:
    raw = str(row.get(key) or '').strip()
    if not raw:
        return default
    return int(raw)


def _build_summary_row(
    *,
    label: str,
    scenario: str,
    n: int,
    seed_base: int,
    cohort: str,
    mc_label: str,
    launch_args: str,
    out_dir: Path,
    summary: dict,
) -> dict[str, str]:
    per_run_csv = out_dir / f'{mc_label}.csv'
    return {
        'label': label,
        'scenario': scenario,
        'n': str(n),
        'seed_base': str(seed_base),
        'cohort': cohort,
        'mc_label': mc_label,
        'n_runs': str(summary.get('n_runs', '')),
        'success_rate_pct': f"{100.0 * float(summary.get('success_rate', 0.0)):.1f}",
        'miss_mean_m': f"{float(summary.get('miss_distance_m', {}).get('mean', float('nan'))):.3f}",
        'miss_p95_m': f"{float(summary.get('miss_distance_m', {}).get('p95', float('nan'))):.3f}",
        'intercept_time_mean_s': f"{float(summary.get('intercept_time_s', {}).get('mean', float('nan'))):.3f}",
        'launch_args': launch_args,
        'summary_json': str((out_dir / f'{mc_label}.json').resolve()),
        'per_run_csv': str(per_run_csv.resolve()),
    }


def _dry_run_summary(n: int) -> dict:
    return {
        'n_runs': n,
        'success_rate': 0.0,
        'miss_distance_m': {'mean': float('nan'), 'p95': float('nan')},
        'intercept_time_s': {'mean': float('nan')},
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profiles-csv', type=Path, default=FIXTURE)
    ap.add_argument('--n', type=int, default=2, help='Default runs per profile when CSV n is empty.')
    ap.add_argument(
        '--override-n',
        type=int,
        default=None,
        help='If set, use this run count for every profile (pilot override; ignores CSV n).',
    )
    ap.add_argument('--seed-base', type=int, default=6201, help='Default seed base when CSV seed_base is empty.')
    ap.add_argument('--timeout-s', type=float, default=75.0)
    ap.add_argument('--study', default='statistical_validation')
    ap.add_argument('--out-dir', type=Path, default=REPO / 'runs' / 'mc')
    ap.add_argument(
        '--out-csv',
        type=Path,
        default=REPO / 'runs' / 'evaluation' / 'statistical_validation_summary.csv',
    )
    ap.add_argument(
        '--headless',
        action='store_true',
        help='Append use_gazebo_gui:=false when not already present in launch_args.',
    )
    ap.add_argument(
        '--dry-run',
        action='store_true',
        help='Print monte_carlo commands only; write summary CSV with placeholder metrics.',
    )
    args = ap.parse_args()

    if not args.profiles_csv.is_file():
        print(f'missing profiles CSV: {args.profiles_csv}', file=sys.stderr)
        return 2

    with args.profiles_csv.open(encoding='utf-8', newline='') as f:
        profiles = list(csv.DictReader(f))

    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.out_csv.parent.mkdir(parents=True, exist_ok=True)

    rows_out: list[dict[str, str]] = []
    for prof in profiles:
        label = str(prof.get('label') or '').strip()
        if not label:
            continue
        scenario = str(prof.get('scenario') or 'single').strip() or 'single'
        n = (
            int(args.override_n)
            if args.override_n is not None
            else _int_field(prof, 'n', args.n)
        )
        seed_base = _int_field(prof, 'seed_base', args.seed_base)
        launch_args = str(prof.get('launch_args') or '').strip()
        if args.headless and 'use_gazebo_gui:=' not in launch_args:
            launch_args = f'{launch_args} use_gazebo_gui:=false'.strip()

        cohort = f'{args.study}_{label}'
        mc_label = f'{args.study}_{label}_n{n}_s{seed_base}'
        cmd = [
            sys.executable,
            'scripts/monte_carlo.py',
            'run',
            '--n',
            str(n),
            '--seed-base',
            str(seed_base),
            '--scenario',
            scenario,
            '--timeout-s',
            str(args.timeout_s),
            '--label',
            mc_label,
            '--out-dir',
            str(args.out_dir),
            '--cohort',
            cohort,
            '--launch-args',
            launch_args,
        ]
        print(f'[statistical_validation] label={label} scenario={scenario} cohort={cohort}', flush=True)
        _run_cmd(cmd, dry_run=args.dry_run)

        if args.dry_run:
            summary = _dry_run_summary(n)
        else:
            summary = _load_json(args.out_dir / f'{mc_label}.json')

        rows_out.append(
            _build_summary_row(
                label=label,
                scenario=scenario,
                n=n,
                seed_base=seed_base,
                cohort=cohort,
                mc_label=mc_label,
                launch_args=launch_args,
                out_dir=args.out_dir,
                summary=summary,
            )
        )

    if not rows_out:
        print('no profiles to run', file=sys.stderr)
        return 2

    with args.out_csv.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(SUMMARY_FIELDS))
        w.writeheader()
        w.writerows(rows_out)
    print(f'[statistical_validation] wrote {args.out_csv}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
