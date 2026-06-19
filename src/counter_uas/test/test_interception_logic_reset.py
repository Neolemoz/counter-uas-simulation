from __future__ import annotations

import ast
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_LOGIC_NODE = _REPO_ROOT / "src" / "gazebo_target_sim" / "gazebo_target_sim" / "interception_logic_node.py"


def _method_source(method_name: str) -> str:
    source = _LOGIC_NODE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "InterceptionLogicNode"
    )
    method = next(
        node for node in cls.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )
    return ast.get_source_segment(source, method) or ""


def test_gazebo_reset_clears_cached_pose_and_guidance_memory() -> None:
    """The reset callback must not let a new run consume stale poses or filtered guidance."""
    source = _method_source("_on_gz_sim_reset")

    for snippet in (
        "self._target = None",
        "self._target_filter_velocity = None",
        "self._inter_pos = {i: None for i in self._ids}",
        "self._inter_start_pos.clear()",
        "self._t_go_filtered = {i: None for i in self._ids}",
        "self._guidance_unit_prev.clear()",
    ):
        assert snippet in source


def test_assignment_clear_drops_guidance_memory_for_next_engagement() -> None:
    source = _method_source("_clear_assignments")

    assert "self._t_go_filtered[iid] = None" in source
    assert "self._guidance_unit_prev.pop(iid, None)" in source
