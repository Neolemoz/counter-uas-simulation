"""Layer B realism contracts that should not drift silently."""

from __future__ import annotations

from pathlib import Path


REPO = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (REPO / path).read_text(encoding='utf-8')


def test_full_stack_bringup_prefers_tracks_state() -> None:
    text = _read('src/counter_uas/launch/bringup.launch.py')

    assert "'intercept_measurement_source'," in text
    assert "default_value='tracks_state'" in text
    assert "'tracks_state_topic': LaunchConfiguration('tracks_state_topic')" in text
    assert "default_value='/tracks/state'" in text


def test_standalone_single_target_keeps_ground_truth_default_with_state_override() -> None:
    text = _read('src/gazebo_target_sim/launch/gazebo_target.launch.py')

    assert "'intercept_measurement_source'," in text
    assert "default_value='ground_truth'" in text
    assert "default_value='/drone/position'" in text
    assert "'tracks_state_topic'," in text
    assert "Odometry topic when intercept_measurement_source:=tracks_state" in text


def test_multi_target_remains_point_based_ground_truth_by_default() -> None:
    text = _read('src/gazebo_target_sim/launch/gazebo_target_multi.launch.py')

    assert "'intercept_measurement_source'," in text
    assert "default_value='ground_truth'" in text
    assert "ignored when multi_target_enabled" in text
    assert "default_value='/tracks'" in text
    assert "'tracks_state_topic'," not in text


def test_offline_heatmap_rollout_defaults_match_live_launch_limits() -> None:
    heatmap = _read('scripts/render_intercept_heatmap_prob_offline.py')
    launch = _read('src/gazebo_target_sim/launch/gazebo_target.launch.py')

    assert "'interceptor_max_turn_rate_rad_s'" in launch
    assert "default_value='2.5'" in launch
    assert "'interceptor_max_accel_m_s2'" in launch
    assert "default_value='30.0'" in launch
    assert "'--rollout-max-turn-rate-rad-s'" in heatmap
    assert 'default=2.5' in heatmap
    assert "'--rollout-max-accel-m-s2'" in heatmap
    assert 'default=30.0' in heatmap
