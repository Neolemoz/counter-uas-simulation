"""Regression checks for launch/config defaults that gate end-to-end engagement."""

from __future__ import annotations

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def test_default_bringup_config_contains_km_scale_tracking_gates() -> None:
    text = (_REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config.yaml').read_text(encoding='utf-8')

    for snippet in (
        'candidate_match_gate_m: 20.0',
        'candidate_predictive_gate: true',
        'association_gate_m: 25.0',
        'confirmation_hits: 2',
        'candidate_max_missed_frames: 5',
        'max_track_speed_mps: 80.0',
        'max_update_jump_m: 25.0',
    ):
        assert snippet in text


def test_multi_launch_relaxes_ground_start_hit_gates() -> None:
    text = (
        _REPO_ROOT
        / 'src'
        / 'gazebo_target_sim'
        / 'launch'
        / 'gazebo_target_multi.launch.py'
    ).read_text(encoding='utf-8')

    assert "'hit_min_interceptor_z_m': 0.05" in text
    assert "'hit_min_interceptor_travel_m': 1.0" in text
    assert "'hit_min_target_z_m': -1.0" in text
    assert "'aim_strike_on_mid_shell': False" in text
