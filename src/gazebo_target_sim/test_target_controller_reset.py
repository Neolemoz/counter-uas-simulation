#!/usr/bin/env python3
"""Regression tests for target reset after explosion removal."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import types
import unittest


class _Parameter:
    def __init__(self, value: object) -> None:
        self.value = value


class _Logger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def info(self, msg: str) -> None:
        self.messages.append(('info', msg))

    def warning(self, msg: str) -> None:
        self.messages.append(('warning', msg))


class _Timer:
    pass


class _Clock:
    def now(self):
        return types.SimpleNamespace(nanoseconds=123)


def _install_ros_stubs() -> None:
    if 'geometry_msgs.msg' not in sys.modules:
        geometry_msgs = types.ModuleType('geometry_msgs')
        geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

        class Point:
            def __init__(self) -> None:
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

        geometry_msgs_msg.Point = Point
        geometry_msgs.msg = geometry_msgs_msg
        sys.modules['geometry_msgs'] = geometry_msgs
        sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    if 'std_msgs.msg' not in sys.modules:
        std_msgs = types.ModuleType('std_msgs')
        std_msgs_msg = types.ModuleType('std_msgs.msg')

        class Bool:
            def __init__(self) -> None:
                self.data = False

        class ColorRGBA:
            def __init__(self, **kwargs: float) -> None:
                self.__dict__.update(kwargs)

        std_msgs_msg.Bool = Bool
        std_msgs_msg.ColorRGBA = ColorRGBA
        std_msgs.msg = std_msgs_msg
        sys.modules['std_msgs'] = std_msgs
        sys.modules['std_msgs.msg'] = std_msgs_msg

    if 'visualization_msgs.msg' not in sys.modules:
        visualization_msgs = types.ModuleType('visualization_msgs')
        visualization_msgs_msg = types.ModuleType('visualization_msgs.msg')

        class Marker:
            SPHERE = 2
            LINE_STRIP = 4
            ADD = 0

            def __init__(self) -> None:
                self.header = types.SimpleNamespace(stamp=None, frame_id='')
                self.pose = types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(w=0.0),
                )
                self.scale = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.lifetime = types.SimpleNamespace(sec=0)
                self.points = []

        visualization_msgs_msg.Marker = Marker
        visualization_msgs.msg = visualization_msgs_msg
        sys.modules['visualization_msgs'] = visualization_msgs
        sys.modules['visualization_msgs.msg'] = visualization_msgs_msg

    if 'rosgraph_msgs.msg' not in sys.modules:
        rosgraph_msgs = types.ModuleType('rosgraph_msgs')
        rosgraph_msgs_msg = types.ModuleType('rosgraph_msgs.msg')

        class Clock:
            pass

        rosgraph_msgs_msg.Clock = Clock
        rosgraph_msgs.msg = rosgraph_msgs_msg
        sys.modules['rosgraph_msgs'] = rosgraph_msgs
        sys.modules['rosgraph_msgs.msg'] = rosgraph_msgs_msg

    if 'rclpy' not in sys.modules:
        rclpy = types.ModuleType('rclpy')
        rclpy.init = lambda *args, **kwargs: None
        rclpy.spin = lambda *args, **kwargs: None
        rclpy.ok = lambda: False
        rclpy.shutdown = lambda: None
        rclpy.executors = types.SimpleNamespace(ExternalShutdownException=Exception)
        rclpy_node = types.ModuleType('rclpy.node')

        class Node:
            pass

        rclpy_node.Node = Node
        rclpy_time = types.ModuleType('rclpy.time')
        rclpy_time.Time = object
        sys.modules['rclpy'] = rclpy
        sys.modules['rclpy.node'] = rclpy_node
        sys.modules['rclpy.time'] = rclpy_time

    if 'rclpy.qos' not in sys.modules:
        rclpy_qos = types.ModuleType('rclpy.qos')
        rclpy_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
        rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
        rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
        rclpy_qos.QoSProfile = lambda **kwargs: kwargs
        sys.modules['rclpy.qos'] = rclpy_qos

    if 'ament_index_python.packages' not in sys.modules:
        ament_index_python = types.ModuleType('ament_index_python')
        packages = types.ModuleType('ament_index_python.packages')
        packages.get_package_share_directory = lambda package: str(
            Path(__file__).resolve().parent,
        )
        ament_index_python.packages = packages
        sys.modules['ament_index_python'] = ament_index_python
        sys.modules['ament_index_python.packages'] = packages


def _load_target_controller_module():
    _install_ros_stubs()
    package_dir = Path(__file__).resolve().parent / 'gazebo_target_sim'
    package = types.ModuleType('gazebo_target_sim')
    package.__path__ = [str(package_dir)]
    sys.modules['gazebo_target_sim'] = package

    for name in ('clock_reset', 'gz_entity_tools', 'gz_pose_tools'):
        spec = importlib.util.spec_from_file_location(
            f'gazebo_target_sim.{name}',
            package_dir / f'{name}.py',
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[f'gazebo_target_sim.{name}'] = module
        spec.loader.exec_module(module)

    spec = importlib.util.spec_from_file_location(
        'gazebo_target_sim.target_controller_node',
        package_dir / 'target_controller_node.py',
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules['gazebo_target_sim.target_controller_node'] = module
    spec.loader.exec_module(module)
    return module


class TargetControllerResetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = _load_target_controller_module()

    def _controller(self):
        node = self.module.TargetControllerNode.__new__(self.module.TargetControllerNode)
        node._logger = _Logger()
        node._params = {
            'start_x_m': _Parameter(-11.0),
            'start_y_m': _Parameter(9.0),
            'start_z_m': _Parameter(40.0),
        }
        node._model = 'sphere_target_0'
        node._px = 1.0
        node._py = 2.0
        node._pz = 3.0
        node._stopped = True
        node._explosion_fade_timer = _Timer()
        node._explosion_fx_name = 'hit_exp_1'
        node._timer = None
        node._motion_rate_hz = 10.0
        node._last_tick_time = object()
        node._last_log_time = object()
        node._target_removed_for_explosion = True
        node._orbit_enabled = False
        node._trail = []
        node.respawn_calls = 0
        node.destroyed_timers = []
        node.created_timers = []

        def get_parameter(name: str) -> _Parameter:
            return node._params[name]

        def destroy_timer(timer: _Timer) -> None:
            node.destroyed_timers.append(timer)

        def create_timer(period: float, callback):
            del callback
            node.created_timers.append(period)
            return _Timer()

        def get_logger() -> _Logger:
            return node._logger

        def get_clock() -> _Clock:
            return _Clock()

        def respawn() -> bool:
            node.respawn_calls += 1
            return True

        node.get_parameter = get_parameter
        node.destroy_timer = destroy_timer
        node.create_timer = create_timer
        node.get_logger = get_logger
        node.get_clock = get_clock
        node._respawn_target_model = respawn
        return node

    def test_reset_respawns_target_removed_by_explosion(self) -> None:
        node = self._controller()

        self.module.TargetControllerNode._on_gz_sim_reset(node)

        self.assertFalse(node._stopped)
        self.assertEqual((node._px, node._py, node._pz), (-11.0, 9.0, 40.0))
        self.assertIsNone(node._explosion_fade_timer)
        self.assertIsNone(node._explosion_fx_name)
        self.assertEqual(node.respawn_calls, 1)
        self.assertFalse(node._target_removed_for_explosion)
        self.assertIsNotNone(node._timer)
        self.assertEqual(node.created_timers, [0.1])

    def test_reset_does_not_respawn_when_target_was_not_removed(self) -> None:
        node = self._controller()
        node._target_removed_for_explosion = False

        self.module.TargetControllerNode._on_gz_sim_reset(node)

        self.assertEqual(node.respawn_calls, 0)

    def test_reset_keeps_removed_flag_when_respawn_fails(self) -> None:
        node = self._controller()
        node._respawn_target_model = lambda: False

        self.module.TargetControllerNode._on_gz_sim_reset(node)

        self.assertTrue(node._target_removed_for_explosion)


if __name__ == '__main__':
    unittest.main()
