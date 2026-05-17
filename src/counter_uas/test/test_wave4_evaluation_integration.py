from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_module(rel_path: str, name: str):  # noqa: ANN201
    path = _REPO_ROOT / rel_path
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_observer_visibility_summary_is_additive_and_backward_compatible() -> None:
    sa = _load_module('scripts/evaluation/selection_audit.py', 'selection_audit_wave4')

    empty = sa.observer_visibility_summary('plain log with no observer markers')
    assert empty == {
        'observer_visibility_windows': 0,
        'persistence_window_count': 0,
        'churn_event_count': 0,
        'selection_proxy_event_count': 0,
    }

    log = """
    [lifecycle_observer_node-1] [LIFECYCLE_OBSERVER] event=summary tracks_state_msgs_window=3 tracks_state_msgs_total=3 unique_track_ids_window=2 track_id_changes_window=1 track_gap_events_window=1 track_persistence_events_window=2 selected_id_changes_window=1 idle_s=0.400 last_track_id=8 selected_id=interceptor_2
    [lifecycle_observer_node-1] [TRACK_CONTINUITY] event=track_gap gap_s=0.400 prev_track_id=7 next_track_id=8 frame_id=track_8
    [lifecycle_observer_node-1] [TRACK_CONTINUITY] event=track_id_change prev_track_id=7 next_track_id=8 gap_s=0.400 frame_id=track_8
    [lifecycle_observer_node-1] [SELECTION_PROXY] event=track_persistence_window track_id=7 persistence_s=1.200 track_switch_to=8 frame_id=track_8
    """.strip()

    out = sa.observer_visibility_summary(log)
    assert out == {
        'observer_visibility_windows': 1,
        'persistence_window_count': 2,
        'churn_event_count': 2,
        'selection_proxy_event_count': 1,
    }


def test_evaluation_row_includes_observer_visibility_additively() -> None:
    ev = _load_module('scripts/evaluation/evaluation_row.py', 'evaluation_row_wave4')

    log = """
    [lifecycle_observer_node-1] [LIFECYCLE_OBSERVER] event=summary tracks_state_msgs_window=3 tracks_state_msgs_total=3 unique_track_ids_window=2 track_id_changes_window=1 track_gap_events_window=1 track_persistence_events_window=2 selected_id_changes_window=1 idle_s=0.400 last_track_id=8 selected_id=interceptor_2
    [lifecycle_observer_node-1] [TRACK_CONTINUITY] event=track_gap gap_s=0.400 prev_track_id=7 next_track_id=8 frame_id=track_8
    [lifecycle_observer_node-1] [TRACK_CONTINUITY] event=track_id_change prev_track_id=7 next_track_id=8 gap_s=0.400 frame_id=track_8
    [lifecycle_observer_node-1] [SELECTION_PROXY] event=track_persistence_window track_id=7 persistence_s=1.200 track_switch_to=8 frame_id=track_8
    """.strip()

    log_path = _REPO_ROOT / 'tmp_wave4_evaluation_row.log'
    log_path.write_text(log, encoding='utf-8')
    try:
        row = ev.evaluation_row(log_path)
    finally:
        log_path.unlink(missing_ok=True)

    assert row['observer_visibility_windows'] == 1
    assert row['persistence_window_count'] == 2
    assert row['churn_event_count'] == 2
    assert row['selection_proxy_event_count'] == 1
    assert row['selection_oracle_divergence_class'] == 'D5_inconclusive_visibility_limited'


def test_selection_audit_summary_includes_additive_mismatch_onset_fields() -> None:
    sa = _load_module('scripts/evaluation/selection_audit.py', 'selection_audit_wave7')

    log = """
    [interception_logic_node-1] === Interceptor Selection ===
    [interception_logic_node-1] interceptor_0: feasible=True, tti=1.0
    [interception_logic_node-1] interceptor_2: feasible=True, tti=2.0
    [interception_logic_node-1] selected: interceptor_0
    [noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_start fragmentation_index=1 gap_ticks=3 mode=staggered tick_index=4 seed=77
    [interception_logic_node-1] === Interceptor Selection ===
    [interception_logic_node-1] interceptor_0: feasible=True, tti=4.0
    [interception_logic_node-1] interceptor_2: feasible=True, tti=2.0
    [interception_logic_node-1] selected: interceptor_0
    [interception_logic_node-1] === Interceptor Selection ===
    [interception_logic_node-1] interceptor_0: feasible=True, tti=5.0
    [interception_logic_node-1] interceptor_2: feasible=True, tti=3.0
    [interception_logic_node-1] selected: interceptor_0
    """.strip()

    out = sa.selection_audit_summary(log)

    assert out['n_selection_blocks'] == 3
    assert out['selection_oracle_match_rate'] == 1 / 3
    assert out['first_selection_mismatch_block'] == 2
    assert out['last_selection_mismatch_block'] == 3
    assert out['selection_mismatch_count'] == 2
    assert out['mismatch_after_fragmented_gap_count'] == 2
