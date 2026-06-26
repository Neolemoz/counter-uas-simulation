"""Regression tests for launch/config defaults with large user-facing blast radius."""

from __future__ import annotations

from pathlib import Path

import yaml


_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_counter_uas_config(name: str) -> dict:
    path = _REPO_ROOT / 'src' / 'counter_uas' / 'config' / name
    assert path.is_file(), f'missing config file: {path}'
    with path.open('r', encoding='utf-8') as fh:
        data = yaml.safe_load(fh)
    assert isinstance(data, dict), f'{path} did not parse to a mapping'
    return data


def _tracking_params(config_name: str) -> dict:
    config = _load_counter_uas_config(config_name)
    params = config.get('tracking_node', {}).get('ros__parameters', {})
    assert isinstance(params, dict), f'{config_name} tracking_node.ros__parameters missing'
    return params


def test_default_bringup_tracking_gates_match_km_scale_tracks_state_path() -> None:
    """Default bringup uses /tracks/state, so config.yaml must not keep lab-scale gates.

    The legacy 1 m candidate gate and zero miss tolerance let the fusion pipeline publish
    detections while no track ever confirmed; downstream interception then idled forever.
    """
    params = _tracking_params('config.yaml')

    assert params['candidate_match_gate_m'] >= 20.0
    assert params['candidate_predictive_gate'] is True
    assert params['association_gate_m'] >= 25.0
    assert params['confirmation_hits'] <= 2
    assert params['candidate_max_missed_frames'] >= 5
    assert params['max_track_speed_mps'] >= 80.0
    assert params['max_update_jump_m'] >= 25.0


def test_default_and_gazebo_configs_keep_tracking_gates_aligned() -> None:
    """The default config is the one bringup.launch.py selects; keep it Gazebo-scale."""
    default = _tracking_params('config.yaml')
    gazebo = _tracking_params('config_gazebo_counter_uas.yaml')

    keys = (
        'candidate_match_gate_m',
        'candidate_predictive_gate',
        'association_gate_m',
        'confirmation_hits',
        'candidate_max_missed_frames',
        'max_track_speed_mps',
        'max_update_jump_m',
    )
    assert {key: default[key] for key in keys} == {key: gazebo[key] for key in keys}
