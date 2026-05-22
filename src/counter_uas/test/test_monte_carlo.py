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


def _write_meta(log_path: Path, *, cohort: str = '', notes: str = '') -> None:
    payload = {
        'cohort': cohort,
        'notes': notes,
    }
    log_path.with_suffix('.meta.json').write_text(json.dumps(payload), encoding='utf-8')


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
    miss = summary['miss_distance_m']
    assert miss['n'] == 2
    assert miss['min'] < 0.5
    assert miss['max'] > 7.0


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
    csv_text = csv_path.read_text(encoding='utf-8')
    assert 'run_id' in csv_text and 'miss_distance_m' in csv_text
    assert 'run_a' in csv_text and 'run_b' in csv_text


def test_aggregate_filters_by_meta_cohort_and_notes(tmp_path) -> None:
    mc = _load_mc()
    logs_dir = tmp_path / 'logs'
    logs_dir.mkdir()
    _write_synthetic_logs(logs_dir)
    _write_meta(logs_dir / 'run_a.log', cohort='cohort_a', notes='arm=A seed=101')
    _write_meta(logs_dir / 'run_b.log', cohort='cohort_b', notes='arm=B seed=101')
    out_dir = tmp_path / 'mc'

    class Args:
        pass

    args = Args()
    args.logs_dir = str(logs_dir)
    args.pattern = '*.log'
    args.label = 'unit'
    args.out_dir = str(out_dir)
    args.meta_cohort = 'cohort_a'
    args.notes_substring = 'arm=A'

    rc = mc.cmd_aggregate(args)
    assert rc == 0
    payload = json.loads((out_dir / 'unit.json').read_text(encoding='utf-8'))
    assert payload['n_runs'] == 1
    csv_text = (out_dir / 'unit.csv').read_text(encoding='utf-8')
    assert 'run_a' in csv_text
    assert 'run_b' not in csv_text


def test_run_mode_source_preserves_seed_geometry_and_cohort_contracts() -> None:
    source = (_REPO_ROOT / 'scripts' / 'monte_carlo.py').read_text(encoding='utf-8')

    assert 'seed = args.seed_base + i' in source
    assert 'noise_seed:={seed}' in source
    assert 'result["noise_seed_mc"] = seed' in source
    assert 'result["seed"] = seed  # backwards compat alias' in source
    assert 'result["geometry_id"] = gid_str' in source
    assert "cmd.extend(['--cohort', str(args.cohort).strip()])" in source


def test_reporting_language_keeps_mc_statistics_descriptive() -> None:
    source = (_REPO_ROOT / 'scripts' / 'monte_carlo.py').read_text(encoding='utf-8')
    readme = (_REPO_ROOT / 'scripts' / 'evaluation' / 'README.md').read_text(encoding='utf-8')

    assert 'report descriptive summaries' in source
    assert 'outputs are descriptive simulation statistics' in source
    assert 'They are not confidence' in source
    assert 'Current summaries include run count, success count/rate' in readme
    assert 'they are not confidence intervals' in readme
    assert 'Use `compare` tables as side-by-side descriptive comparisons' in readme
