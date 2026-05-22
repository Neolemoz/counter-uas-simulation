from __future__ import annotations

import importlib.util
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_realism_metrics():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'realism_metrics.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('realism_metrics', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_realism_metrics_counts_additive_events_and_track_recovery() -> None:
    mod = _load_realism_metrics()
    log = """
    [noisy_measurement_node-1] [REALISM_EVENT] ghost_detection count=1 offset_xy_m=6.500 placement=near_threshold tick_index=7 seed=77
    [noisy_measurement_node-1] [REALISM_EVENT] dropout_burst_start burst_index=1 burst_ticks=3
    [noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_start fragmentation_index=1 gap_ticks=2 mode=staggered tick_index=8 cycle_ticks=6 phase_ticks=1 seed=77
    [noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_end mode=staggered tick_index=9 seed=77
    [noisy_measurement_node-1] [REALISM_EVENT] delayed_detection count=1 delay_s=0.100
    [noisy_measurement_node-1] [REALISM_EVENT] stale_detection count=1 source_age_s=0.200
    [tracking_node-1] Candidate detected: position=(1.000, 2.000, 3.000) hit_count=1
    [tracking_node-1] Candidate discarded: position=(1.000, 2.000, 3.000) hit_count was 1 missed_frames=2
    [tracking_node-1] Merging track 1 and track 2
    [tracking_node-1] Track 3 NOT updated (missed_frames=1)
    [tracking_node-1] Track 3 NOT updated (missed_frames=2)
    [tracking_node-1] Track 3 updated (valid match)
    [tracking_node-1] Track 3 NOT updated (missed_frames=1)
    [tracking_node-1] Track 3 updated (valid match)
    [tracking_node-1] Track 9 NOT updated (missed_frames=1)
    [tracking_node-1] Track 3 updated (valid match)
    """.strip()

    out = mod.realism_metrics_summary(log)

    assert out['ghost_detection_count'] == 1
    assert out['dropout_burst_count'] == 1
    assert out['fragmented_gap_count'] == 1
    assert out['fragmented_gap_end_count'] == 1
    assert out['delayed_detection_count'] == 1
    assert out['stale_detection_count'] == 1
    assert out['candidate_spawn_count'] == 1
    assert out['candidate_discard_count'] == 1
    assert out['duplicate_track_merge_count'] == 1
    assert out['track_coast_count'] == 4
    assert out['track_recovery_count'] == 2
    assert out['max_track_missed_frames'] == 2
    assert out['coast_recovery_cycle_count'] == 2
    assert out['recovery_thrash_count'] == 1
    assert out['unrecovered_coast_track_count'] == 1
    assert out['candidate_churn_pressure_count'] == 2
    assert out['lifecycle_thrash_score'] == 4


def test_realism_metrics_handles_logs_without_realism_tags() -> None:
    mod = _load_realism_metrics()
    out = mod.realism_metrics_summary('plain log with no realism markers')
    assert out == {
        'delayed_detection_count': 0,
        'stale_detection_count': 0,
        'dropout_burst_count': 0,
        'ghost_detection_count': 0,
        'fragmented_gap_count': 0,
        'fragmented_gap_end_count': 0,
        'lifecycle_observer_summary_count': 0,
        'tracks_state_observation_count': 0,
        'unique_track_ids_observed_count': 0,
        'track_continuity_gap_count': 0,
        'track_continuity_change_count': 0,
        'selection_proxy_window_count': 0,
        'selection_proxy_event_count': 0,
        'track_persistence_boundary_count': 0,
        'lifecycle_observer_selected_id_count': 0,
        'candidate_spawn_count': 0,
        'candidate_discard_count': 0,
        'duplicate_track_merge_count': 0,
        'track_coast_count': 0,
        'track_recovery_count': 0,
        'max_track_missed_frames': 0,
        'coast_recovery_cycle_count': 0,
        'recovery_thrash_count': 0,
        'unrecovered_coast_track_count': 0,
        'candidate_churn_pressure_count': 0,
        'lifecycle_thrash_score': 0,
    }


def test_realism_metrics_accepts_wave1_annotation_fields_without_contract_change() -> None:
    mod = _load_realism_metrics()
    log = """
    [noisy_measurement_node-1] [REALISM_EVENT] ghost_detection count=4 offset_xy_m=19.250 placement=near_threshold tick_index=13 seed=5101
    [noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_start fragmentation_index=2 gap_ticks=2 mode=random tick_index=15 seed=5101
    [noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_end mode=random tick_index=16 seed=5101
    """.strip()

    out = mod.realism_metrics_summary(log)

    assert out['ghost_detection_count'] == 1
    assert out['fragmented_gap_count'] == 1
    assert out['fragmented_gap_end_count'] == 1


def test_realism_metrics_counts_passive_lifecycle_observer_evidence_additively() -> None:
    mod = _load_realism_metrics()
    log = """
    [lifecycle_observer_node-1] [LIFECYCLE_OBSERVER] event=armed tracks_state_topic=/tracks/state selected_id_topic=/interceptor/selected_id summary_period_s=5.000 gap_warn_s=0.300
    [lifecycle_observer_node-1] [TRACK_CONTINUITY] event=track_first_seen track_id=7 frame_id=track_7
    [lifecycle_observer_node-1] [TRACK_CONTINUITY] event=track_gap gap_s=0.400 prev_track_id=7 next_track_id=8 frame_id=track_8
    [lifecycle_observer_node-1] [TRACK_CONTINUITY] event=track_id_change prev_track_id=7 next_track_id=8 gap_s=0.400 frame_id=track_8
    [lifecycle_observer_node-1] [LIFECYCLE_OBSERVER] event=selected_id selected_id=interceptor_2 previous_selected_id=(none)
    [lifecycle_observer_node-1] [SELECTION_PROXY] event=track_persistence_window track_id=7 persistence_s=1.200 track_switch_to=8 frame_id=track_8
    [lifecycle_observer_node-1] [TRACK_PERSISTENCE] event=track_persistence_boundary previous_track_id=7 next_track_id=8 persistence_s=1.200
    [lifecycle_observer_node-1] [LIFECYCLE_OBSERVER] event=summary tracks_state_msgs_window=4 tracks_state_msgs_total=4 unique_track_ids_window=2 track_id_changes_window=1 track_gap_events_window=1 track_persistence_events_window=1 selected_id_changes_window=1 idle_s=0.500 last_track_id=8 selected_id=interceptor_2
    """.strip()

    out = mod.realism_metrics_summary(log)

    assert out['lifecycle_observer_summary_count'] == 1
    assert out['tracks_state_observation_count'] == 4
    assert out['unique_track_ids_observed_count'] == 2
    assert out['track_continuity_gap_count'] == 1
    assert out['track_continuity_change_count'] == 1
    assert out['selection_proxy_window_count'] == 1
    assert out['selection_proxy_event_count'] == 1
    assert out['track_persistence_boundary_count'] == 1
    assert out['lifecycle_observer_selected_id_count'] == 1
