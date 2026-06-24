"""Regression checks for launch/config defaults that gate full-stack engagement."""

from __future__ import annotations

import ast
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def test_default_config_has_gazebo_scale_tracking_gates() -> None:
    cfg = (_REPO_ROOT / "src" / "counter_uas" / "config" / "config.yaml").read_text(encoding="utf-8")

    for expected in (
        "candidate_match_gate_m: 20.0",
        "candidate_predictive_gate: true",
        "association_gate_m: 25.0",
        "confirmation_hits: 2",
        "candidate_max_missed_frames: 5",
        "max_track_speed_mps: 80.0",
        "max_update_jump_m: 25.0",
    ):
        assert expected in cfg


def test_interceptor_controller_uses_shared_norm_helper() -> None:
    path = _REPO_ROOT / "src" / "gazebo_target_sim" / "gazebo_target_sim" / "interceptor_controller_node.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))

    imported = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module == "gazebo_target_sim.kinematic_plant"
        for alias in node.names
    }
    attrs = {
        node.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self"
    }

    assert "norm3" in imported
    assert "_norm3" not in attrs
