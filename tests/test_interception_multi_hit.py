from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path


def _install_ros_stubs() -> None:
    geom = types.ModuleType('geometry_msgs')
    geom_msg = types.ModuleType('geometry_msgs.msg')

    class Point:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
            self.x = x
            self.y = y
            self.z = z

    class Vector3(Point):
        pass

    geom_msg.Point = Point
    geom_msg.Vector3 = Vector3
    geom.msg = geom_msg

    rclpy = types.ModuleType('rclpy')
    rclpy.executors = types.SimpleNamespace(ExternalShutdownException=Exception)
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None
    rclpy_node = types.ModuleType('rclpy.node')

    class Node:
        pass

    rclpy_node.Node = Node
    rclpy_qos = types.ModuleType('rclpy.qos')
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.QoSProfile = object
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    rclpy_time = types.ModuleType('rclpy.time')

    class Time:
        pass

    rclpy_time.Time = Time

    std = types.ModuleType('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')

    class Bool:
        def __init__(self, data: bool = False) -> None:
            self.data = data

    class ColorRGBA:
        pass

    class String:
        def __init__(self, data: str = '') -> None:
            self.data = data

    std_msg.Bool = Bool
    std_msg.ColorRGBA = ColorRGBA
    std_msg.String = String
    std.msg = std_msg

    viz = types.ModuleType('visualization_msgs')
    viz_msg = types.ModuleType('visualization_msgs.msg')

    class Marker:
        pass

    viz_msg.Marker = Marker
    viz.msg = viz_msg

    pkg = types.ModuleType('gazebo_target_sim')
    clock_reset = types.ModuleType('gazebo_target_sim.clock_reset')
    clock_reset.subscribe_sim_time_reset = lambda node, callback: None
    pkg.clock_reset = clock_reset

    sys.modules.update(
        {
            'geometry_msgs': geom,
            'geometry_msgs.msg': geom_msg,
            'rclpy': rclpy,
            'rclpy.node': rclpy_node,
            'rclpy.qos': rclpy_qos,
            'rclpy.time': rclpy_time,
            'std_msgs': std,
            'std_msgs.msg': std_msg,
            'visualization_msgs': viz,
            'visualization_msgs.msg': viz_msg,
            'gazebo_target_sim': pkg,
            'gazebo_target_sim.clock_reset': clock_reset,
        },
    )


def _load_interception_module():
    _install_ros_stubs()
    path = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'gazebo_target_sim'
        / 'gazebo_target_sim'
        / 'interception_logic_node.py'
    )
    spec = importlib.util.spec_from_file_location('interception_logic_node_under_test', path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MultiTargetHitTest(unittest.TestCase):
    def test_multi_target_hit_does_not_pause_world_until_all_targets_hit(self) -> None:
        module = _load_interception_module()

        node = object.__new__(module.InterceptionLogicNode)
        node._hits_multi = {'target_0': False, 'target_1': False}
        node._multi_labels = ['target_0', 'target_1']
        node._inter_pos = {'interceptor_0': module.Point(x=0.0, y=0.0, z=0.0)}
        node._hit_min_tz = None
        node._hit_thresh = 1.0
        node._min_miss_distance = float('inf')
        node._pause_gz_on_hit = True
        node._paused = False
        node._stopped: list[str | None] = []

        node._record_miss_distance = lambda miss_distance: None
        node._publish_stop_signal = lambda target_label: node._stopped.append(target_label)
        node._log_active_targets_remaining = lambda: None
        node._pause_gazebo_world = lambda: setattr(node, '_paused', True)

        hit = node._multi_try_hit(0.2, 0.0, 0.0, 'target_0', 'interceptor_0', 'select', strike_ok=True)

        self.assertIs(hit, True)
        self.assertEqual(node._hits_multi, {'target_0': True, 'target_1': False})
        self.assertEqual(node._stopped, ['target_0'])
        self.assertIs(node._paused, False)

        hit = node._multi_try_hit(0.2, 0.0, 0.0, 'target_1', 'interceptor_0', 'select', strike_ok=True)

        self.assertIs(hit, True)
        self.assertEqual(node._hits_multi, {'target_0': True, 'target_1': True})
        self.assertEqual(node._stopped, ['target_0', 'target_1'])
        self.assertIs(node._paused, True)


if __name__ == '__main__':
    unittest.main()
