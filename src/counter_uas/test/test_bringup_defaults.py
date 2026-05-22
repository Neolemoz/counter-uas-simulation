"""Regression coverage for default full-stack bringup configuration."""

from __future__ import annotations

import re
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_CONFIG = _REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config.yaml'


def _scalar_param(text: str, name: str) -> str:
    match = re.search(rf'^\s+{re.escape(name)}:\s*([^\n#]+)', text, re.MULTILINE)
    assert match is not None, f'missing {name} in default config.yaml'
    return match.group(1).strip()


def _float_param(text: str, name: str) -> float:
    return float(_scalar_param(text, name))


def test_default_config_has_km_scale_tracking_gates_for_tracks_state() -> None:
    text = _DEFAULT_CONFIG.read_text(encoding='utf-8')

    assert _float_param(text, 'candidate_match_gate_m') >= 20.0
    assert _scalar_param(text, 'candidate_predictive_gate') == 'true'
    assert _float_param(text, 'association_gate_m') >= 25.0
    assert int(_scalar_param(text, 'confirmation_hits')) <= 2
    assert int(_scalar_param(text, 'candidate_max_missed_frames')) >= 5
    assert _float_param(text, 'max_track_speed_mps') >= 80.0
    assert _float_param(text, 'max_update_jump_m') >= 25.0
