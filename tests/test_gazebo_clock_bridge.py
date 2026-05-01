from __future__ import annotations

import ast
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
LAUNCH_FILES = [
    WORKSPACE / "src/gazebo_target_sim/launch/gazebo_target.launch.py",
    WORKSPACE / "src/gazebo_target_sim/launch/gazebo_target_multi.launch.py",
]


def _clock_bridge_nodes(source: str) -> list[ast.Call]:
    tree = ast.parse(source)
    nodes: list[ast.Call] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(target, ast.Name) and target.id == "clock_bridge" for target in node.targets):
            continue
        if isinstance(node.value, ast.Call):
            nodes.append(node.value)
    return nodes


def _keyword(call: ast.Call, name: str) -> ast.AST:
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    raise AssertionError(f"missing keyword {name!r}")


def test_gazebo_clock_bridge_remaps_world_clock_to_clock() -> None:
    for launch_file in LAUNCH_FILES:
        nodes = _clock_bridge_nodes(launch_file.read_text(encoding="utf-8"))
        assert len(nodes) == 1, f"{launch_file} should define one clock bridge"

        arguments = ast.unparse(_keyword(nodes[0], "arguments"))
        remappings = ast.unparse(_keyword(nodes[0], "remappings"))

        assert "/world/{world_name}/clock" in arguments
        assert "/world/{world_name}/clock" in remappings
        assert "'/clock'" in remappings or '"/clock"' in remappings
