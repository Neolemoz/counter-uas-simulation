"""Regression checks for launch/config defaults that do not require ROS imports."""

from __future__ import annotations

import re
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _flat_yaml_value(text: str, key: str) -> str:
    match = re.search(rf"^\s*{re.escape(key)}:\s*(\S+)\s*$", text, flags=re.MULTILINE)
    assert match is not None, f"missing {key}"
    return match.group(1)


def test_default_config_has_gazebo_scale_tracking_gates() -> None:
    """Default bringup feeds /tracks/state, so config.yaml must confirm km-scale tracks."""
    default_text = (_REPO_ROOT / "src" / "counter_uas" / "config" / "config.yaml").read_text(
        encoding="utf-8",
    )
    gazebo_text = (
        _REPO_ROOT / "src" / "counter_uas" / "config" / "config_gazebo_counter_uas.yaml"
    ).read_text(encoding="utf-8")

    keys = (
        "candidate_match_gate_m",
        "candidate_predictive_gate",
        "association_gate_m",
        "confirmation_hits",
        "candidate_max_missed_frames",
        "max_track_speed_mps",
        "max_update_jump_m",
    )
    for key in keys:
        assert _flat_yaml_value(default_text, key) == _flat_yaml_value(gazebo_text, key)


def test_default_bringup_uses_track_state_feed() -> None:
    launch_text = (_REPO_ROOT / "src" / "counter_uas" / "launch" / "bringup.launch.py").read_text(
        encoding="utf-8",
    )
    assert "default_value='tracks_state'" in launch_text
