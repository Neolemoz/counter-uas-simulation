"""Regression: scripts/monte_carlo.aggregate produces correct stats from synthetic logs."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

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


def _cmd_run_args(tmp_path: Path, *, n: int, launch_args: str | None = None):  # noqa: ANN201
    class Args:
        pass

    args = Args()
    args.n = n
    args.seed_base = 10
    args.geometry_id = ''
    args.scenario = 'single'
    args.timeout_s = 1.0
    args.launch_args = launch_args
    args.label = 'unit_run'
    args.out_dir = str(tmp_path / 'mc')
    args.cohort = ''
    return args


def test_cmd_run_replaces_launch_noise_seed_with_per_run_seed(tmp_path, monkeypatch) -> None:
    mc = _load_mc()
    logs_dir = tmp_path / 'logs'
    logs_dir.mkdir()
    seen_launch_args: list[str] = []

    def fake_run(cmd, capture_output, text):  # noqa: ANN001, ANN202
        idx = cmd.index('--launch-args')
        launch_args = cmd[idx + 1]
        seen_launch_args.append(launch_args)
        seed = launch_args.split('noise_seed:=', 1)[1].split()[0]
        log_path = logs_dir / f'run_{seed}.log'
        log_path.write_text(_HIT_LOG, encoding='utf-8')
        log_path.with_suffix('.meta.json').write_text(
            json.dumps({'launch_args_raw': launch_args, 'launch_args_kv': {'noise_seed': seed}}),
            encoding='utf-8',
        )
        return SimpleNamespace(returncode=0, stdout=f'{log_path}\n', stderr='')

    monkeypatch.setattr(mc.subprocess, 'run', fake_run)

    args = _cmd_run_args(tmp_path, n=2, launch_args='use_gazebo_gui:=false noise_seed:=999')
    rc = mc.cmd_run(args)

    assert rc == 0
    assert seen_launch_args == [
        'use_gazebo_gui:=false noise_seed:=10',
        'use_gazebo_gui:=false noise_seed:=11',
    ]
    csv_text = (tmp_path / 'mc' / 'unit_run.csv').read_text(encoding='utf-8')
    assert ',10,10,' in csv_text
    assert ',11,11,' in csv_text
    assert '999' not in csv_text


def test_cmd_run_fails_nonzero_for_partial_cohort(tmp_path, monkeypatch) -> None:
    mc = _load_mc()
    logs_dir = tmp_path / 'logs'
    logs_dir.mkdir()
    calls = 0

    def fake_run(cmd, capture_output, text):  # noqa: ANN001, ANN202
        nonlocal calls
        calls += 1
        if calls == 2:
            return SimpleNamespace(returncode=2, stdout='', stderr='sim launch failed')
        idx = cmd.index('--launch-args')
        seed = cmd[idx + 1].split('noise_seed:=', 1)[1].split()[0]
        log_path = logs_dir / f'run_{seed}.log'
        log_path.write_text(_HIT_LOG, encoding='utf-8')
        return SimpleNamespace(returncode=0, stdout=f'{log_path}\n', stderr='')

    monkeypatch.setattr(mc.subprocess, 'run', fake_run)

    args = _cmd_run_args(tmp_path, n=2)
    rc = mc.cmd_run(args)

    assert rc == 1
    payload = json.loads((tmp_path / 'mc' / 'unit_run.json').read_text(encoding='utf-8'))
    assert payload['n_runs'] == 1
    assert payload['n_requested'] == 2
    assert payload['n_skipped'] == 1
    assert payload['skipped_runs'][0]['reason'] == 'run_capture_failed'
