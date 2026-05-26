from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_PKG_ROOT = _REPO_ROOT / "src" / "gazebo_target_sim"
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))


def _load_interceptor_module():  # noqa: ANN201
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.shutdown = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.spin = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.ok = MagicMock(return_value=False)  # type: ignore[attr-defined]
    rclpy_mod.executors = types.SimpleNamespace(ExternalShutdownException=RuntimeError)  # type: ignore[attr-defined]
    rclpy_node_mod = types.ModuleType("rclpy.node")
    rclpy_node_mod.Node = object  # type: ignore[attr-defined]
    rclpy_time_mod = types.ModuleType("rclpy.time")
    rclpy_time_mod.Time = object  # type: ignore[attr-defined]

    geom_mod = types.ModuleType("geometry_msgs.msg")

    class _Point:
        __slots__ = ("x", "y", "z")

        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Quaternion:
        def __init__(self, *, x=0.0, y=0.0, z=0.0, w=1.0) -> None:  # noqa: ANN001
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class _Vector3(_Point):
        pass

    geom_mod.Point = _Point  # type: ignore[attr-defined]
    geom_mod.Quaternion = _Quaternion  # type: ignore[attr-defined]
    geom_mod.Vector3 = _Vector3  # type: ignore[attr-defined]
    geom_pkg = types.ModuleType("geometry_msgs")
    geom_pkg.msg = geom_mod  # type: ignore[attr-defined]

    std_mod = types.ModuleType("std_msgs.msg")

    class _String:
        def __init__(self) -> None:
            self.data = ""

    class _ColorRGBA:
        def __init__(self, *, r=0.0, g=0.0, b=0.0, a=0.0) -> None:  # noqa: ANN001
            self.r = r
            self.g = g
            self.b = b
            self.a = a

    std_mod.String = _String  # type: ignore[attr-defined]
    std_mod.ColorRGBA = _ColorRGBA  # type: ignore[attr-defined]
    std_pkg = types.ModuleType("std_msgs")
    std_pkg.msg = std_mod  # type: ignore[attr-defined]

    viz_mod = types.ModuleType("visualization_msgs.msg")

    class _Marker:
        CUBE = 1
        ADD = 0

        def __init__(self) -> None:
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.pose = types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=None,
            )
            self.scale = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.color = None
            self.ns = ""
            self.id = 0
            self.type = 0
            self.action = 0

    viz_mod.Marker = _Marker  # type: ignore[attr-defined]
    viz_pkg = types.ModuleType("visualization_msgs")
    viz_pkg.msg = viz_mod  # type: ignore[attr-defined]

    impact_mod = types.ModuleType("gazebo_target_sim_interfaces.msg")

    class _ImpactEvent:
        def __init__(self) -> None:
            self.interceptor_id = ""

    impact_mod.ImpactEvent = _ImpactEvent  # type: ignore[attr-defined]
    impact_pkg = types.ModuleType("gazebo_target_sim_interfaces")
    impact_pkg.msg = impact_mod  # type: ignore[attr-defined]

    clock_reset_mod = types.ModuleType("gazebo_target_sim.clock_reset")
    clock_reset_mod.subscribe_sim_time_reset = MagicMock()  # type: ignore[attr-defined]

    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = rclpy_node_mod
    sys.modules["rclpy.time"] = rclpy_time_mod
    sys.modules["geometry_msgs"] = geom_pkg
    sys.modules["geometry_msgs.msg"] = geom_mod
    sys.modules["std_msgs"] = std_pkg
    sys.modules["std_msgs.msg"] = std_mod
    sys.modules["visualization_msgs"] = viz_pkg
    sys.modules["visualization_msgs.msg"] = viz_mod
    sys.modules["gazebo_target_sim_interfaces"] = impact_pkg
    sys.modules["gazebo_target_sim_interfaces.msg"] = impact_mod
    sys.modules["gazebo_target_sim.clock_reset"] = clock_reset_mod

    path = _PKG_ROOT / "gazebo_target_sim" / "interceptor_controller_node.py"
    spec = importlib.util.spec_from_file_location("interceptor_controller_node_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_quat_from_motion_uses_shared_norm_helper_without_attribute_error() -> None:
    mod = _load_interceptor_module()
    node = object.__new__(mod.InterceptorControllerNode)
    node._v_orient_floor = 0.08

    qx, qy, qz, qw = mod.InterceptorControllerNode._quat_from_motion(node, 0.0, 2.0, 0.0, False)

    assert (qx, qy, qz, qw) == pytest.approx((0.0, 0.0, 0.70710678, 0.70710678))


def test_startup_origin_reset_is_destroyed_and_syncs_internal_state() -> None:
    mod = _load_interceptor_module()
    node = object.__new__(mod.InterceptorControllerNode)
    timer = object()
    destroyed: list[object] = []
    poses: list[tuple[float, float, float, float, float, float, float]] = []
    params = {
        "origin_x": -15.0,
        "origin_y": 2.0,
        "origin_z": 0.5,
    }

    node._origin_reset_timer = timer
    node._idle = True
    node._impact_hidden = False
    node._px = 123.0
    node._py = 456.0
    node._pz = 789.0
    node._vx_s = 1.0
    node._vy_s = 2.0
    node._vz_s = 3.0
    node._dt = 0.1
    node._last_timer_time = object()
    node.destroy_timer = destroyed.append
    node.get_parameter = lambda name: types.SimpleNamespace(value=params[name])
    node._plant_params = lambda dt: mod.KinematicPlantParams(dt_s=dt, max_speed_m_s=20.0)
    node._call_set_pose = lambda *args: poses.append(args) or True

    mod.InterceptorControllerNode._reset_to_origin_once(node)
    mod.InterceptorControllerNode._reset_to_origin_once(node)

    assert destroyed == [timer]
    assert poses == [(-15.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0)]
    assert node._origin_reset_timer is None
    assert (node._px, node._py, node._pz) == (-15.0, 2.0, 0.5)
    assert (node._vx_s, node._vy_s, node._vz_s) == (0.0, 0.0, 0.0)
    assert node._plant_state.position == (-15.0, 2.0, 0.5)
    assert node._last_timer_time is None
