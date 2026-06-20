"""Regression tests for default launch/config wiring."""

from __future__ import annotations

import re
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[3]


def _yaml_scalar(text: str, key: str) -> str:
    match = re.search(rf'^\s*{re.escape(key)}:\s*([^\n#]+)', text, flags=re.MULTILINE)
    assert match, f'missing {key}'
    return match.group(1).strip()


def test_default_config_uses_km_scale_tracking_gates() -> None:
    text = (_REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config.yaml').read_text(encoding='utf-8')
    assert float(_yaml_scalar(text, 'candidate_match_gate_m')) >= 20.0
    assert _yaml_scalar(text, 'candidate_predictive_gate').lower() == 'true'
    assert float(_yaml_scalar(text, 'association_gate_m')) >= 25.0
    assert int(float(_yaml_scalar(text, 'confirmation_hits'))) <= 2
    assert int(float(_yaml_scalar(text, 'candidate_max_missed_frames'))) >= 5
    assert float(_yaml_scalar(text, 'max_track_speed_mps')) >= 80.0
    assert float(_yaml_scalar(text, 'max_update_jump_m')) >= 25.0
