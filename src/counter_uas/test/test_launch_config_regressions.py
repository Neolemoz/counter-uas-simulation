"""Regression tests for launch/config defaults that can disable interception."""

from __future__ import annotations

import ast
import re
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _read_repo(path: str) -> str:
    p = _REPO_ROOT / path
    assert p.is_file(), f"missing {p}"
    return p.read_text(encoding="utf-8")


def _yaml_scalar(text: str, key: str) -> str:
    m = re.search(rf"^\s*{re.escape(key)}:\s*([^\n#]+)", text, flags=re.MULTILINE)
    assert m, f"missing YAML key {key!r}"
    return m.group(1).strip()


def _launch_literal(text: str, key: str):
    m = re.search(rf"['\"]{re.escape(key)}['\"]\s*:\s*([^,\n]+)", text)
    assert m, f"missing launch param {key!r}"
    return ast.literal_eval(m.group(1).strip())


def test_default_config_tracks_state_gate_is_km_scale() -> None:
    """Default bringup consumes /tracks/state, so config.yaml must not use lab-scale gates."""
    text = _read_repo("src/counter_uas/config/config.yaml")

    assert _yaml_scalar(text, "candidate_match_gate_m") == "20.0"
    assert _yaml_scalar(text, "candidate_predictive_gate") == "true"
    assert _yaml_scalar(text, "association_gate_m") == "25.0"
    assert _yaml_scalar(text, "confirmation_hits") == "2"
    assert _yaml_scalar(text, "candidate_max_missed_frames") == "5"
    assert _yaml_scalar(text, "max_track_speed_mps") == "80.0"
    assert _yaml_scalar(text, "max_update_jump_m") == "25.0"


def test_multi_target_launch_hits_ground_start_targets_directly() -> None:
    """Multi-target defaults must match the fixed single-target ground-start HIT gates."""
    text = _read_repo("src/gazebo_target_sim/launch/gazebo_target_multi.launch.py")

    assert _launch_literal(text, "hit_min_interceptor_z_m") == 0.05
    assert _launch_literal(text, "hit_min_interceptor_travel_m") == 1.0
    assert _launch_literal(text, "hit_min_target_z_m") == -1.0
    assert _launch_literal(text, "aim_strike_on_mid_shell") is False
