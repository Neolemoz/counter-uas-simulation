"""Regressions for launch defaults that wire full-stack interception."""

from __future__ import annotations

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def test_default_config_contains_km_scale_tracking_gates() -> None:
    text = (_REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config.yaml').read_text(encoding='utf-8')
    for expected in (
        'candidate_match_gate_m: 20.0',
        'candidate_predictive_gate: true',
        'association_gate_m: 25.0',
        'confirmation_hits: 2',
        'candidate_max_missed_frames: 5',
        'max_track_speed_mps: 80.0',
        'max_update_jump_m: 25.0',
    ):
        assert expected in text

