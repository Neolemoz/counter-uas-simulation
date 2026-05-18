"""Regression coverage for default full-stack bringup parameters."""

from __future__ import annotations

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _tracking_params_block(config_text: str) -> str:
    lines = config_text.splitlines()
    start = None
    for i, line in enumerate(lines):
        if line == 'tracking_node:':
            start = i
            break
    assert start is not None, 'config.yaml must define tracking_node parameters'

    block: list[str] = []
    for line in lines[start + 1 :]:
        if line and not line.startswith((' ', '#')):
            break
        block.append(line)
    return '\n'.join(block)


def test_default_config_uses_km_scale_tracking_overrides() -> None:
    """Default bringup uses /tracks/state, so its config must actually form tracks at km scale."""
    config_path = _REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config.yaml'
    block = _tracking_params_block(config_path.read_text(encoding='utf-8'))

    expected = {
        'candidate_match_gate_m: 20.0',
        'candidate_predictive_gate: true',
        'association_gate_m: 25.0',
        'confirmation_hits: 2',
        'candidate_max_missed_frames: 5',
        'max_track_speed_mps: 80.0',
        'max_update_jump_m: 25.0',
    }
    missing = sorted(param for param in expected if param not in block)
    assert not missing, f'default config missing km-scale tracker overrides: {missing}'
