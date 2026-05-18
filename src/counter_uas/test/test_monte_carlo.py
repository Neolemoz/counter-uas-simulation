"""Regression: scripts/monte_carlo.aggregate produces correct stats from synthetic logs."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_mc():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'monte_carlo.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('monte_carlo', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


_HIT_LOG = """
[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=12.500 m | t_go=2.5 s | vel=18.0 m/s | mode=predict
[interception_logic_node-1] [HIT] interceptor_0 layer=engage  min_miss=0.45 m  hit_threshold = 1.0 m
""".strip()

_MISS_LOG = """
[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=42.100 m | t_go=8.4 s | vel=22.1 m/s | mode=pursuit
[interception_logic_node-1] [min_miss] = 7.812 m
[interception_logic_node-1] hit_threshold = 1.0 m
""".strip()


def _write_synthetic_logs(tmp_dir: Path) -> None:
    (tmp_dir / 'run_a.log').write_text(_HIT_LOG, encoding='utf-8')
    (tmp_dir / 'run_b.log').write_text(_MISS_LOG, encoding='utf-8')
    (tmp_dir / 'run_a.meta.json').write_text(
        json.dumps(
            {
                'cohort': 'unit_cohort',
                'git_commit': 'abc123',
                'git_dirty': False,
                'notes': 'mc_label=unit seed=100 geometry_id="cell_a"',
                'launch_args_raw': 'use_gazebo_gui:=false noise_seed:=100',
                'launch_args_kv': {'noise_seed': '100'},
            },
        ),
        encoding='utf-8',
    )
    (tmp_dir / 'run_b.meta.json').write_text(
        json.dumps(
            {
                'cohort': 'unit_cohort',
                'git_commit': 'abc123',
                'git_dirty': False,
                'notes': 'mc_label=unit seed=101 geometry_id="cell_a"',
                'launch_args_raw': 'use_gazebo_gui:=false noise_seed:=101',
                'launch_args_kv': {'noise_seed': '101'},
            },
        ),
        encoding='utf-8',
    )


def test_summarise_counts_success_and_misses(tmp_path) -> None:
    mc = _load_mc()
    logs_dir = tmp_path / 'logs'
    logs_dir.mkdir()
    _write_synthetic_logs(logs_dir)

    analyze = mc._load_analyze_run()
    rows = []
    for log in sorted(logs_dir.glob('*.log')):
        r = analyze.parse_run_to_result(str(log))
        r['run_id'] = log.stem
        r['log_path'] = str(log)
        rows.append(r)
    summary = mc._summarise(rows, 'unit')

    assert summary['n_runs'] == 2
    assert summary['n_success'] == 1
    assert summary['success_rate'] == 0.5
    assert summary['success_rate_ci95']['method'] == 'wilson'
    assert summary['cohort_tier'] == 'smoke'
    miss = summary['miss_distance_m']
    assert miss['n'] == 2
    assert miss['min'] < 0.5
    assert miss['max'] > 7.0
    assert 'p95_ci95' in miss


def test_aggregate_writes_outputs(tmp_path) -> None:
    mc = _load_mc()
    logs_dir = tmp_path / 'logs'
    logs_dir.mkdir()
    _write_synthetic_logs(logs_dir)
    out_dir = tmp_path / 'mc'

    class Args:
        pass

    args = Args()
    args.logs_dir = str(logs_dir)
    args.pattern = '*.log'
    args.label = 'unit'
    args.out_dir = str(out_dir)
    args.meta_cohort = ''
    args.notes_substring = ''

    rc = mc.cmd_aggregate(args)
    assert rc == 0
    json_path = out_dir / 'unit.json'
    csv_path = out_dir / 'unit.csv'
    assert json_path.is_file()
    assert csv_path.is_file()
    payload = json.loads(json_path.read_text(encoding='utf-8'))
    assert payload['label'] == 'unit'
    assert payload['n_runs'] == 2
    assert 0.0 <= payload['success_rate'] <= 1.0
    assert payload['success_rate_ci95']['method'] == 'wilson'
    csv_text = csv_path.read_text(encoding='utf-8')
    assert 'run_id' in csv_text and 'miss_distance_m' in csv_text
    assert 'noise_seed_mc' in csv_text and 'cohort' in csv_text and 'meta_path' in csv_text
    assert 'unit_cohort' in csv_text and 'cell_a' in csv_text
    assert 'run_a' in csv_text and 'run_b' in csv_text
