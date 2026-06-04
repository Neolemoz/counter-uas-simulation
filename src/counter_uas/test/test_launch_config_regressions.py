"""Regression tests for launch/config contracts that gate default Gazebo engagement."""

from __future__ import annotations

from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_yaml(path: Path) -> dict:
    with path.open(encoding='utf-8') as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict)
    return data


def test_default_config_includes_km_scale_tracking_gates() -> None:
    cfg = _load_yaml(_REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config.yaml')
    params = cfg['tracking_node']['ros__parameters']

    assert params['candidate_match_gate_m'] >= 20.0
    assert params['candidate_predictive_gate'] is True
    assert params['association_gate_m'] >= 25.0
    assert params['confirmation_hits'] == 2
    assert params['candidate_max_missed_frames'] >= 5
    assert params['max_track_speed_mps'] >= 80.0
    assert params['max_update_jump_m'] >= 25.0


def test_default_config_tracks_gates_match_gazebo_scale_config() -> None:
    default_cfg = _load_yaml(_REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config.yaml')
    gazebo_cfg = _load_yaml(_REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config_gazebo_counter_uas.yaml')
    keys = {
        'candidate_match_gate_m',
        'candidate_predictive_gate',
        'association_gate_m',
        'confirmation_hits',
        'candidate_max_missed_frames',
        'max_track_speed_mps',
        'max_update_jump_m',
    }
    default_params = default_cfg['tracking_node']['ros__parameters']
    gazebo_params = gazebo_cfg['tracking_node']['ros__parameters']

    assert {k: default_params[k] for k in keys} == {k: gazebo_params[k] for k in keys}
