"""Regression checks for launch/config contracts that do not require ROS imports."""

from __future__ import annotations

import re
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_CONFIG_DIR = _REPO_ROOT / 'src' / 'counter_uas' / 'config'


def _scalar_param(path: Path, name: str) -> str:
    text = path.read_text(encoding='utf-8')
    match = re.search(rf'^\s*{re.escape(name)}:\s*(\S+)\s*$', text, flags=re.MULTILINE)
    assert match is not None, f'{name} missing from {path}'
    return match.group(1)


def test_default_config_has_km_scale_tracking_gates() -> None:
    default = _CONFIG_DIR / 'config.yaml'
    gazebo = _CONFIG_DIR / 'config_gazebo_counter_uas.yaml'
    critical_params = [
        'candidate_match_gate_m',
        'candidate_predictive_gate',
        'association_gate_m',
        'confirmation_hits',
        'candidate_max_missed_frames',
        'max_track_speed_mps',
        'max_update_jump_m',
    ]

    for name in critical_params:
        assert _scalar_param(default, name) == _scalar_param(gazebo, name)
