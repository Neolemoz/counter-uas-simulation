"""Regression tests for default full-stack bringup configuration."""

from __future__ import annotations

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _read_config(name: str) -> str:
    path = _REPO_ROOT / 'src' / 'counter_uas' / 'config' / name
    assert path.is_file(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_default_config_has_km_scale_tracking_gates() -> None:
    text = _read_config('config.yaml')
    required = {
        'candidate_match_gate_m: 20.0',
        'candidate_predictive_gate: true',
        'association_gate_m: 25.0',
        'confirmation_hits: 2',
        'candidate_max_missed_frames: 5',
        'max_track_speed_mps: 80.0',
        'max_update_jump_m: 25.0',
    }
    missing = sorted(item for item in required if item not in text)
    assert not missing, f'default config is missing Gazebo-scale tracking gates: {missing}'


def test_default_bringup_uses_tracks_state() -> None:
    launch = _REPO_ROOT / 'src' / 'counter_uas' / 'launch' / 'bringup.launch.py'
    text = launch.read_text(encoding='utf-8')
    assert "default_value='tracks_state'" in text
