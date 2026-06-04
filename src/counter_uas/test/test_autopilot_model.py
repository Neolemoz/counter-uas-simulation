"""Phase 5: smoke tests for the autopilot delay/bandwidth model.

We can't easily import ``InterceptorControllerNode`` (it constructs a ROS node), so we test the
mathematical core (FIFO delay + first-order bandwidth) by re-implementing the same recurrence
the node uses.  Any drift in the node's logic should be caught by porting the change here.
"""

from __future__ import annotations

from collections import deque
import importlib.util
import sys
import types
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_CONTROLLER_STUB_MODULES = (
    'rclpy',
    'rclpy.node',
    'rclpy.time',
    'rclpy.qos',
    'geometry_msgs',
    'geometry_msgs.msg',
    'std_msgs',
    'std_msgs.msg',
    'visualization_msgs',
    'visualization_msgs.msg',
    'gazebo_target_sim_interfaces',
    'gazebo_target_sim_interfaces.msg',
    'rosgraph_msgs',
    'rosgraph_msgs.msg',
)


def _delay_fifo(buf: deque, cmd: tuple[float, float, float]) -> tuple[float, float, float]:
    """Mirror ``InterceptorControllerNode._apply_cmd_delay`` for testing."""
    if len(buf) == 0:
        return cmd
    buf.append(cmd)
    return buf.popleft()


def _first_order_step(prev: tuple[float, float, float], cmd: tuple[float, float, float], dt: float, tau: float) -> tuple[float, float, float]:
    a = dt / (dt + tau)
    return (
        prev[0] + a * (cmd[0] - prev[0]),
        prev[1] + a * (cmd[1] - prev[1]),
        prev[2] + a * (cmd[2] - prev[2]),
    )


def _install_controller_import_stubs() -> dict[str, types.ModuleType | None]:
    previous = {name: sys.modules.get(name) for name in _CONTROLLER_STUB_MODULES}

    class _Dummy:
        def __init__(self, *args, **kwargs) -> None:  # noqa: ANN002, ANN003
            pass

    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.ok = lambda: False
    rclpy.executors = types.SimpleNamespace(ExternalShutdownException=Exception)
    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = object
    rclpy_time = types.ModuleType('rclpy.time')
    rclpy_time.Time = _Dummy
    rclpy_qos = types.ModuleType('rclpy.qos')
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=0)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=0)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=0)
    rclpy_qos.QoSProfile = _Dummy

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msgs_msg.Point = _Dummy
    geometry_msgs_msg.Quaternion = _Dummy
    geometry_msgs_msg.Vector3 = _Dummy

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.String = _Dummy
    std_msgs_msg.ColorRGBA = _Dummy

    visualization_msgs = types.ModuleType('visualization_msgs')
    visualization_msgs_msg = types.ModuleType('visualization_msgs.msg')
    visualization_msgs_msg.Marker = _Dummy

    interfaces = types.ModuleType('gazebo_target_sim_interfaces')
    interfaces_msg = types.ModuleType('gazebo_target_sim_interfaces.msg')
    interfaces_msg.ImpactEvent = _Dummy
    rosgraph_msgs = types.ModuleType('rosgraph_msgs')
    rosgraph_msgs_msg = types.ModuleType('rosgraph_msgs.msg')
    rosgraph_msgs_msg.Clock = _Dummy

    sys.modules.update(
        {
            'rclpy': rclpy,
            'rclpy.node': rclpy_node,
            'rclpy.time': rclpy_time,
            'rclpy.qos': rclpy_qos,
            'geometry_msgs': geometry_msgs,
            'geometry_msgs.msg': geometry_msgs_msg,
            'std_msgs': std_msgs,
            'std_msgs.msg': std_msgs_msg,
            'visualization_msgs': visualization_msgs,
            'visualization_msgs.msg': visualization_msgs_msg,
            'gazebo_target_sim_interfaces': interfaces,
            'gazebo_target_sim_interfaces.msg': interfaces_msg,
            'rosgraph_msgs': rosgraph_msgs,
            'rosgraph_msgs.msg': rosgraph_msgs_msg,
        },
    )
    return previous


def _restore_controller_import_stubs(previous: dict[str, types.ModuleType | None]) -> None:
    for name, module in previous.items():
        if module is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


def _load_controller_module():  # noqa: ANN201
    previous = _install_controller_import_stubs()
    src_dir = _REPO_ROOT / 'src' / 'gazebo_target_sim'
    if str(src_dir) not in sys.path:
        sys.path.insert(0, str(src_dir))
    path = src_dir / 'gazebo_target_sim' / 'interceptor_controller_node.py'
    spec = importlib.util.spec_from_file_location('interceptor_controller_node_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    try:
        spec.loader.exec_module(mod)
    finally:
        _restore_controller_import_stubs(previous)
    return mod


def test_delay_fifo_zero_length_passthrough() -> None:
    buf: deque = deque()
    assert _delay_fifo(buf, (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


def test_delay_fifo_fixed_lag() -> None:
    """A 3-step FIFO seeded with zeros emits the input three steps later."""
    buf = deque([(0.0, 0.0, 0.0)] * 3)  # no maxlen, matches the node implementation
    out0 = _delay_fifo(buf, (1.0, 0.0, 0.0))
    out1 = _delay_fifo(buf, (2.0, 0.0, 0.0))
    out2 = _delay_fifo(buf, (3.0, 0.0, 0.0))
    out3 = _delay_fifo(buf, (4.0, 0.0, 0.0))
    assert out0 == (0.0, 0.0, 0.0)
    assert out1 == (0.0, 0.0, 0.0)
    assert out2 == (0.0, 0.0, 0.0)
    assert out3 == (1.0, 0.0, 0.0)


def test_first_order_reaches_step_input() -> None:
    """Step input drives the response asymptotically toward the command."""
    state = (0.0, 0.0, 0.0)
    tau = 0.2
    dt = 0.01
    target = (10.0, 0.0, 0.0)
    for _ in range(int(5 * tau / dt)):  # 5 time constants
        state = _first_order_step(state, target, dt, tau)
    # After 5*tau the response should be > 99% of target.
    assert state[0] == pytest.approx(target[0], rel=0.02)


def test_first_order_63_pct_at_one_tau() -> None:
    """First-order tracking reaches ~63% of a step at t = tau (canonical bandwidth check)."""
    state = (0.0, 0.0, 0.0)
    tau = 0.5
    dt = 0.01
    target = (1.0, 0.0, 0.0)
    n = int(round(tau / dt))
    for _ in range(n):
        state = _first_order_step(state, target, dt, tau)
    # Discrete approximation of 1 - exp(-1) ≈ 0.6321; allow a small numerical margin for the
    # forward-Euler discretisation used in the node.
    assert 0.55 < state[0] < 0.7


def test_quat_from_motion_uses_shared_norm_without_node_crash() -> None:
    mod = _load_controller_module()
    node = mod.InterceptorControllerNode.__new__(mod.InterceptorControllerNode)
    node._v_orient_floor = 0.1

    assert node._quat_from_motion(0.0, 0.0, 0.0, idle=False) == (0.0, 0.0, 0.0, 1.0)
    q = node._quat_from_motion(1.0, 0.0, 0.0, idle=False)
    assert q == pytest.approx((0.0, 0.0, 0.0, 1.0))
