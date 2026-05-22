"""Regression: new tactical observability tags stay parser-safe."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile

import pytest

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / 'scripts' / 'evaluation'
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from classify_run import classify_run_failure  # noqa: E402
from selection_audit import extract_selection_blocks, selection_audit_summary  # noqa: E402


def _load_analyze_run():  # noqa: ANN201
    path = _REPO / 'scripts' / 'analyze_run.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('analyze_run', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


_SAMPLE_LOG = r"""
[interception_logic_node-1] [TACTICAL_COMMIT] event=commit previous=(none) selected_id=interceptor_0 reason=feasibility_based_initial_commit t_rel_s=12.500000
[interception_logic_node-1] [TACTICAL_SELECTED_ID] topic=/interceptor/selected_id selected_id=interceptor_0 previous=(none) transition=commit meaning=committed_interceptor_for_single_target reason=feasibility_based_selected_commit
[interception_logic_node-1] [FEASIBILITY] feasible=True  t_intercept@cap=3.200s  required_min_speed=20.000 m/s  interceptor_cap=55.000 m/s  selected='interceptor_0'  d_threat=1200.000 m
[interception_logic_node-1] [TACTICAL_FEASIBILITY] selected_id=interceptor_0 feasible=true reason=feasible t_intercept_at_cap_s=3.200000 required_min_speed_m_s=20.000000 d_threat_m=1200.000000
[interception_logic_node-1] [TACTICAL_FEASIBILITY_REJECT] interceptor=interceptor_1 selected=false reason=required_min_speed_above_cap t_intercept_at_cap_s=(none) required_min_speed_m_s=72.000000
[interception_logic_node-1] === Interceptor Selection ===
[interception_logic_node-1] interceptor_0: feasible=True, tti=3.2
[interception_logic_node-1] interceptor_1: feasible=False, tti=n/a
[interception_logic_node-1] selected: interceptor_0
[interception_logic_node-1] [TACTICAL_SWITCH] decision=hold reason=tti_margin_not_met current=interceptor_0 candidate=interceptor_1 current_tti_s=3.200000 candidate_tti_s=3.100000 margin_s=0.500000 dwell_s=0.100000 switch_window_s=1.000000
[interception_logic_node-1] [TACTICAL_ASSIGNMENT_LOCK] previous=(none) assigned_interceptor=interceptor_0 reason=switch_evaluation_complete meaning=assignment_lock_window_for_dome_off_committed_selection lock_duration_s=1.500000
[interception_logic_node-1] [TACTICAL_ASSIGNED_TARGET] interceptor=interceptor_0 assigned_target=target_0 previous=(none) transition=commit meaning=non_empty_arms_multi_target_motion_empty_disarms reason=assigned_target_commit
[interception_logic_node-1] [TACTICAL_MULTI_ASSIGN] target=target_0 assigned_interceptor=interceptor_0 decision=stable_keep_previous previous=interceptor_0 greedy=interceptor_1 prev_tti_s=3.200000 greedy_tti_s=2.900000 switch_margin_s=0.500000
[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=1200.500 m | t_go=45.2 s | vel=55.0 m/s | mode=predict
[interception_logic_node-1] [HIT] min_miss=0.42 m layer=detect
""".strip()


def test_selection_audit_ignores_tactical_observability_lines() -> None:
    blocks = extract_selection_blocks(_SAMPLE_LOG)
    assert len(blocks) == 1
    assert blocks[0]['selected'] == 'interceptor_0'
    assert blocks[0]['oracle_match'] is True

    summary = selection_audit_summary(_SAMPLE_LOG)
    assert summary['n_selection_blocks'] == 1
    assert summary['last_selected'] == 'interceptor_0'
    assert summary['last_oracle_match'] is True


def test_classify_run_does_not_treat_observability_switch_as_assignment_failure() -> None:
    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False, encoding='utf-8') as f:
        f.write(_SAMPLE_LOG + '\n')
        p = Path(f.name)
    try:
        assert classify_run_failure(p) == 'F5_unknown'
    finally:
        p.unlink(missing_ok=True)


def test_analyze_run_ignores_tactical_observability_lines() -> None:
    ar = _load_analyze_run()
    data = ar.parse_log(_SAMPLE_LOG)
    assert data['hit'] is True
    assert data['min_miss_m'] == pytest.approx(0.42)
    assert data['dist_series'] == [pytest.approx(1200.5)]
    assert data['tgo_series'] == [pytest.approx(45.2)]
