from __future__ import annotations

from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[3]
_CONFIG_DIR = _REPO_ROOT / "src" / "counter_uas" / "config"


def _tracking_params(config_name: str) -> dict:
    data = yaml.safe_load((_CONFIG_DIR / config_name).read_text())
    return data["tracking_node"]["ros__parameters"]


def test_default_km_config_uses_gazebo_tracking_gates() -> None:
    """Default bringup is km-scale; keep its tracking gates aligned with the Gazebo profile."""
    default = _tracking_params("config.yaml")
    gazebo = _tracking_params("config_gazebo_counter_uas.yaml")

    for key in (
        "candidate_match_gate_m",
        "candidate_predictive_gate",
        "association_gate_m",
        "confirmation_hits",
        "candidate_max_missed_frames",
        "max_track_speed_mps",
        "max_update_jump_m",
    ):
        assert default[key] == gazebo[key]
