"""Regression coverage for launch/config defaults that affect engagement."""

from __future__ import annotations

from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _tracking_params(config_name: str) -> dict[str, object]:
    path = _REPO_ROOT / 'src' / 'counter_uas' / 'config' / config_name
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    return dict(data['tracking_node']['ros__parameters'])


def test_default_config_uses_gazebo_scale_tracking_gates() -> None:
    default = _tracking_params('config.yaml')
    gazebo = _tracking_params('config_gazebo_counter_uas.yaml')
    required_keys = [
        'candidate_match_gate_m',
        'candidate_predictive_gate',
        'association_gate_m',
        'confirmation_hits',
        'candidate_max_missed_frames',
        'max_track_speed_mps',
        'max_update_jump_m',
    ]
    for key in required_keys:
        assert default[key] == gazebo[key]
