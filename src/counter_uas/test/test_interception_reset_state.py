"""Regression checks for sim-reset cache clearing in interception logic.

The ROS node is not importable in the lightweight unit-test environment, so this
uses AST inspection to lock the reset invariants that prevent stale pose caches
from creating immediate false HITs after a Gazebo clock rewind.
"""

from __future__ import annotations

import ast
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_NODE_PATH = (
    _REPO_ROOT
    / 'src'
    / 'gazebo_target_sim'
    / 'gazebo_target_sim'
    / 'interception_logic_node.py'
)


def _reset_method() -> ast.FunctionDef:
    tree = ast.parse(_NODE_PATH.read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == 'InterceptionLogicNode':
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == '_on_gz_sim_reset':
                    return item
    raise AssertionError('InterceptionLogicNode._on_gz_sim_reset not found')


def _is_self_attr(node: ast.AST, name: str) -> bool:
    return (
        isinstance(node, ast.Attribute)
        and node.attr == name
        and isinstance(node.value, ast.Name)
        and node.value.id == 'self'
    )


def _assigns_none_to_attr(method: ast.FunctionDef, name: str) -> bool:
    for node in ast.walk(method):
        if (
            not isinstance(node, ast.Assign)
            or not isinstance(node.value, ast.Constant)
            or node.value.value is not None
        ):
            continue
        if any(_is_self_attr(target, name) for target in node.targets):
            return True
    return False


def test_sim_reset_clears_stale_single_target_pose_cache() -> None:
    method = _reset_method()
    assert _assigns_none_to_attr(method, '_target')
    assert _assigns_none_to_attr(method, '_target_filter_velocity')


def test_sim_reset_rearms_interceptor_launch_anchors() -> None:
    method = _reset_method()
    clears_start_pos = False
    clears_current_inter_pos = False

    for node in ast.walk(method):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == 'clear'
            and _is_self_attr(node.func.value, '_inter_start_pos')
        ):
            clears_start_pos = True
        if (
            isinstance(node, ast.Assign)
            and isinstance(node.value, ast.Constant)
            and node.value.value is None
            and any(
                isinstance(target, ast.Subscript) and _is_self_attr(target.value, '_inter_pos')
                for target in node.targets
            )
        ):
            clears_current_inter_pos = True

    assert clears_current_inter_pos
    assert clears_start_pos
