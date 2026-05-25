"""Regression coverage for default bringup configuration."""

from __future__ import annotations

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_CONFIG_DIR = _REPO_ROOT / 'src' / 'counter_uas' / 'config'


def _parse_scalar(value: str):  # noqa: ANN201
    value = value.strip()
    if value.lower() == 'true':
        return True
    if value.lower() == 'false':
        return False
    try:
        if any(ch in value for ch in ('.', 'e', 'E')):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _tracking_params(config_name: str) -> dict[str, object]:
    path = _CONFIG_DIR / config_name
    params: dict[str, object] = {}
    in_tracking = False
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('tracking_node:'):
            in_tracking = True
            continue
        if in_tracking and line and not line.startswith(' '):
            break
        if not in_tracking:
            continue
        stripped = line.strip()
        if not stripped or stripped.startswith('#') or ': ' not in stripped:
            continue
        key, value = stripped.split(': ', 1)
        params[key] = _parse_scalar(value)
    return params


def test_default_config_preserves_km_scale_tracking_gates() -> None:
    default = _tracking_params('config.yaml')
    gazebo = _tracking_params('config_gazebo_counter_uas.yaml')
    required_keys = {
        'candidate_match_gate_m',
        'candidate_predictive_gate',
        'association_gate_m',
        'confirmation_hits',
        'candidate_max_missed_frames',
        'max_track_speed_mps',
        'max_update_jump_m',
    }

    assert required_keys <= default.keys()
    assert {key: default[key] for key in required_keys} == {
        key: gazebo[key] for key in required_keys
    }
