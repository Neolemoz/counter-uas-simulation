"""Regression coverage for interceptor controller motion helpers."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_PKG_ROOT = _REPO_ROOT / 'src' / 'gazebo_target_sim'


def _install_ros_stubs(monkeypatch: pytest.MonkeyPatch) -> None:
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_node_mod = types.ModuleType('rclpy.node')
    rclpy_time_mod = types.ModuleType('rclpy.time')
    rclpy_qos_mod = types.ModuleType('rclpy.qos')
    rclpy_node_mod.Node = type('Node', (), {})  # type: ignore[attr-defined]
    rclpy_time_mod.Time = type('Time', (), {})  # type: ignore[attr-defined]
    for name in ('DurabilityPolicy', 'HistoryPolicy', 'ReliabilityPolicy'):
        setattr(rclpy_qos_mod, name, types.SimpleNamespace(BEST_EFFORT=1, VOLATILE=1, KEEP_LAST=1))
    rclpy_qos_mod.QoSProfile = lambda **_kwargs: object()  # type: ignore[attr-defined]

    geom_pkg = types.ModuleType('geometry_msgs')
    geom_msg = types.ModuleType('geometry_msgs.msg')
    for name in ('Point', 'Quaternion', 'Vector3'):
        setattr(geom_msg, name, type(name, (), {}))

    std_pkg = types.ModuleType('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')
    for name in ('String', 'ColorRGBA'):
        setattr(std_msg, name, type(name, (), {}))

    viz_pkg = types.ModuleType('visualization_msgs')
    viz_msg = types.ModuleType('visualization_msgs.msg')
    viz_msg.Marker = type('Marker', (), {})

    iface_pkg = types.ModuleType('gazebo_target_sim_interfaces')
    iface_msg = types.ModuleType('gazebo_target_sim_interfaces.msg')
    iface_msg.ImpactEvent = type('ImpactEvent', (), {})

    rosgraph_pkg = types.ModuleType('rosgraph_msgs')
    rosgraph_msg = types.ModuleType('rosgraph_msgs.msg')
    rosgraph_msg.Clock = type('Clock', (), {})

    for name, mod in {
        'rclpy': rclpy_mod,
        'rclpy.node': rclpy_node_mod,
        'rclpy.time': rclpy_time_mod,
        'rclpy.qos': rclpy_qos_mod,
        'geometry_msgs': geom_pkg,
        'geometry_msgs.msg': geom_msg,
        'std_msgs': std_pkg,
        'std_msgs.msg': std_msg,
        'visualization_msgs': viz_pkg,
        'visualization_msgs.msg': viz_msg,
        'gazebo_target_sim_interfaces': iface_pkg,
        'gazebo_target_sim_interfaces.msg': iface_msg,
        'rosgraph_msgs': rosgraph_pkg,
        'rosgraph_msgs.msg': rosgraph_msg,
    }.items():
        monkeypatch.setitem(sys.modules, name, mod)


def _load_controller(monkeypatch: pytest.MonkeyPatch):  # noqa: ANN201
    _install_ros_stubs(monkeypatch)
    monkeypatch.syspath_prepend(str(_PKG_ROOT))
    path = _PKG_ROOT / 'gazebo_target_sim' / 'interceptor_controller_node.py'
    spec = importlib.util.spec_from_file_location('interceptor_controller_node_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_quat_from_motion_handles_non_idle_velocity(monkeypatch: pytest.MonkeyPatch) -> None:
    mod = _load_controller(monkeypatch)
    node = object.__new__(mod.InterceptorControllerNode)
    node._v_orient_floor = 0.08

    qx, qy, qz, qw = mod.InterceptorControllerNode._quat_from_motion(node, 1.0, 0.0, 0.0, False)

    assert (qx, qy, qz, qw) == pytest.approx((0.0, 0.0, 0.0, 1.0))
