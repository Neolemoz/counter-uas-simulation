"""Regression checks for launch/config defaults that gate engagement."""

from __future__ import annotations

from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[3]
_CONFIG = _REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config.yaml'
_GAZEBO_CONFIG = _REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config_gazebo_counter_uas.yaml'


def _tracking_params(path: Path) -> dict[str, str]:
    params: dict[str, str] = {}
    in_tracking = False
    in_ros_params = False
    for line in path.read_text(encoding='utf-8').splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        if not line.startswith(' ') and stripped.endswith(':'):
            in_tracking = stripped == 'tracking_node:'
            in_ros_params = False
            continue
        if in_tracking and line.startswith('  ') and not line.startswith('    ') and stripped.endswith(':'):
            in_ros_params = stripped == 'ros__parameters:'
            continue
        if in_tracking and in_ros_params and line.startswith('    ') and ':' in stripped:
            key, value = stripped.split(':', 1)
            params[key.strip()] = value.strip()
    return params


def test_default_config_keeps_km_scale_tracking_gates() -> None:
    default_params = _tracking_params(_CONFIG)
    gazebo_params = _tracking_params(_GAZEBO_CONFIG)
    for key in (
        'candidate_match_gate_m',
        'candidate_predictive_gate',
        'association_gate_m',
        'confirmation_hits',
        'candidate_max_missed_frames',
        'max_track_speed_mps',
        'max_update_jump_m',
    ):
        assert default_params[key] == gazebo_params[key]

