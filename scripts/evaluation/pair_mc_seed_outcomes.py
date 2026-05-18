#!/usr/bin/env python3
"""Join two monte_carlo per-run CSVs by MC noise_seed (from log_path sidecar .meta.json).

Classifies paired rows into buckets G1..G4 matching the PN ablation plan:
  G1: baseline success, candidate fail
  G2: candidate success, baseline fail
  G3: both success, candidate worse miss or intercept time (thresholds configurable)
  G4: both fail
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from statistical_validation import paired_report, seed_for_row  # noqa: E402


def seed_from_meta(log_path: str) -> int | None:
    p = Path(log_path)
    meta = p.with_suffix('.meta.json')
    if not meta.is_file():
        return None
    try:
        data = json.loads(meta.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None
    notes = str(data.get('notes') or '')
    m = re.search(r'\bseed=(\d+)\b', notes)
    if not m:
        return None
    return int(m.group(1))


def load_by_seed(csv_path: Path) -> dict[int, dict[str, object]]:
    out: dict[int, dict[str, object]] = {}
    with csv_path.open(encoding='utf-8', newline='') as f:
        for row in csv.DictReader(f):
            sid = seed_for_row(dict(row))
            if sid is None:
                lp = (row.get('log_path') or '').strip()
                if not lp:
                    continue
                sid = seed_from_meta(lp)
            if sid is None:
                continue
            out[sid] = dict(row)
    return out


def _load_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open(encoding='utf-8', newline='') as f:
        return [dict(r) for r in csv.DictReader(f)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('baseline_csv', type=Path, help='e.g. runs/mc/*baseline*.csv')
    ap.add_argument('candidate_csv', type=Path)
    ap.add_argument(
        '--out-csv',
        type=Path,
        default=None,
        help='Write paired table with bucket column',
    )
    ap.add_argument(
        '--out-json',
        type=Path,
        default=None,
        help='Write Layer C paired statistics JSON with confidence intervals and failure transitions',
    )
    ap.add_argument(
        '--miss-delta',
        type=float,
        default=0.02,
        help='G3: candidate miss worse than baseline by at least this much (m)',
    )
    ap.add_argument(
        '--tint-delta',
        type=float,
        default=0.05,
        help='G3: candidate intercept_time worse by at least this much (s)',
    )
    args = ap.parse_args()

    base_map = load_by_seed(args.baseline_csv)
    cand_map = load_by_seed(args.candidate_csv)
    seeds = sorted(set(base_map) & set(cand_map))
    stats_payload, stats_rows = paired_report(_load_rows(args.baseline_csv), _load_rows(args.candidate_csv))
    stats_by_seed = {int(r['seed']): r for r in stats_rows}

    buckets: defaultdict[str, list[int]] = defaultdict(list)
    rows_out: list[dict[str, object]] = []

    for seed in seeds:
        b = base_map[seed]
        c = cand_map[seed]
        bs = str(b.get('success', '')).strip().lower() == 'true'
        cs = str(c.get('success', '')).strip().lower() == 'true'
        miss_b = float(b.get('miss_distance_m') or 0)
        miss_c = float(c.get('miss_distance_m') or 0)
        tint_b = float(b.get('intercept_time_s') or 0)
        tint_c = float(c.get('intercept_time_s') or 0)

        bucket = ''
        if bs and not cs:
            bucket = 'G1_base_ok_cand_fail'
        elif cs and not bs:
            bucket = 'G2_cand_ok_base_fail'
        elif not bs and not cs:
            bucket = 'G4_both_fail'
        elif bs and cs:
            worse_miss = miss_c > miss_b + args.miss_delta
            worse_tint = tint_c > tint_b + args.tint_delta
            if worse_miss or worse_tint:
                bucket = 'G3_both_ok_cand_worse'
            else:
                bucket = 'G0_both_ok_cand_not_worse'
        else:
            bucket = 'G_unexpected'

        buckets[bucket].append(seed)
        row_out = {
            'seed': seed,
            'bucket': bucket,
            'base_ok': bs,
            'cand_ok': cs,
            'miss_b': miss_b,
            'miss_c': miss_c,
            'miss_delta_c_minus_b': miss_c - miss_b,
            'tint_b': tint_b,
            'tint_c': tint_c,
            'tint_delta_c_minus_b': tint_c - tint_b,
            'failure_b': stats_by_seed.get(seed, {}).get('failure_b', ''),
            'failure_c': stats_by_seed.get(seed, {}).get('failure_c', ''),
            'log_b': b.get('log_path', ''),
            'log_c': c.get('log_path', ''),
        }
        rows_out.append(row_out)

    print('Paired seeds:', len(seeds))
    for name in sorted(buckets):
        print(f'  {name}: {len(buckets[name])}  {buckets[name][:20]}{"..." if len(buckets[name]) > 20 else ""}')
    ps = stats_payload.get('paired_success', {})
    print(
        'Paired success delta:',
        f"{float(ps.get('paired_success_delta', 0.0)):.4f}",
        'CI95=',
        ps.get('paired_success_delta_ci95', {}),
    )

    if args.out_csv:
        args.out_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.out_csv.open('w', encoding='utf-8', newline='') as f:
            w = csv.DictWriter(
                f,
                fieldnames=[
                    'seed',
                    'bucket',
                    'base_ok',
                    'cand_ok',
                    'miss_b',
                    'miss_c',
                    'miss_delta_c_minus_b',
                    'tint_b',
                    'tint_c',
                    'tint_delta_c_minus_b',
                    'failure_b',
                    'failure_c',
                    'log_b',
                    'log_c',
                ],
            )
            w.writeheader()
            w.writerows(rows_out)
        print('Wrote', args.out_csv.resolve())

    if args.out_json:
        stats_payload['legacy_buckets'] = {name: len(vals) for name, vals in sorted(buckets.items())}
        stats_payload['legacy_bucket_seeds'] = {name: vals for name, vals in sorted(buckets.items())}
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        args.out_json.write_text(json.dumps(stats_payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        print('Wrote', args.out_json.resolve())

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
