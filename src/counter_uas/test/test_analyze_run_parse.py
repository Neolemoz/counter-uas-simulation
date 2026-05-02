"""Regression: scripts/analyze_run.parse_log + parse_run_to_result stay stable."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_analyze_run():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'analyze_run.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('analyze_run', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


_GOLDEN_LOG = r"""
[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=1200.500 m | t_go=45.2 s | vel=55.0 m/s | mode=predict
[interception_logic_node-1] [Intercept Debug]
[interception_logic_node-1] distance=1200.500
[interception_logic_node-1] t_hit=12.34
[interception_logic_node-1] cmd_vel=(1.0,2.0,0.5)
[interception_logic_node-1] cmd_vel=(1.1,2.0,0.4)
[interception_logic_node-1] mode=predict
[interception_logic_node-1] [HIT] min_miss=0.42 m layer=detect
""".strip()


def test_parse_log_extracts_series() -> None:
    ar = _load_analyze_run()
    data = ar.parse_log(_GOLDEN_LOG)
    assert data['hit'] is True
    assert data['min_miss_m'] == pytest.approx(0.42)
    assert len(data['dist_series']) == 1
    assert data['dist_series'][0] == pytest.approx(1200.5)
    assert len(data['tgo_series']) == 1
    assert data['tgo_series'][0] == pytest.approx(45.2)
    assert len(data['heading_changes']) == 1


def test_parse_run_to_result_shape(tmp_path: Path) -> None:
    ar = _load_analyze_run()
    p = tmp_path / 'run.log'
    p.write_text(_GOLDEN_LOG + '\n', encoding='utf-8')
    r = ar.parse_run_to_result(str(p))
    assert r['success'] is True
    assert r['miss_distance_m'] == pytest.approx(0.42)
    assert r['intercept_time_s'] == pytest.approx(45.2)


def test_metrics_row_regex_matches_prefixed_line() -> None:
    ar = _load_analyze_run()
    line = (
        '[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=3.400 m | '
        't_go=1.100 s | vel=4.0 m/s | mode=pursuit'
    )
    data = ar.parse_log(line)
    assert data['dist_series'] == [pytest.approx(3.4)]
    assert data['tgo_series'] == [pytest.approx(1.1)]
