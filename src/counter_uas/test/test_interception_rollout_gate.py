from __future__ import annotations

import importlib.util
import random
import sys
import types
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[3]
_PKG_ROOT = _REPO_ROOT / 'src' / 'gazebo_target_sim'


def _stub_msg_package(monkeypatch, pkgname: str, names: list[str]) -> None:  # noqa: ANN001
    msg_mod = types.ModuleType(f'{pkgname}.msg')
    for name in names:
        setattr(msg_mod, name, type(name, (), {'__init__': lambda self: None}))
    pkg_mod = types.ModuleType(pkgname)
    pkg_mod.msg = msg_mod  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, pkgname, pkg_mod)
    monkeypatch.setitem(sys.modules, f'{pkgname}.msg', msg_mod)


def _load_interception_module(monkeypatch):  # noqa: ANN001, ANN201
    if str(_PKG_ROOT) not in sys.path:
        sys.path.insert(0, str(_PKG_ROOT))

    rclpy_mod = types.ModuleType('rclpy')
    duration_mod = types.ModuleType('rclpy.duration')
    node_mod = types.ModuleType('rclpy.node')
    qos_mod = types.ModuleType('rclpy.qos')
    time_mod = types.ModuleType('rclpy.time')

    class _Duration:
        pass

    class _Node:
        pass

    class _Time:
        pass

    duration_mod.Duration = _Duration  # type: ignore[attr-defined]
    node_mod.Node = _Node  # type: ignore[attr-defined]
    time_mod.Time = _Time  # type: ignore[attr-defined]
    for name in ('DurabilityPolicy', 'HistoryPolicy', 'QoSProfile', 'ReliabilityPolicy'):
        setattr(qos_mod, name, type(name, (), {}))
    rclpy_mod.duration = duration_mod  # type: ignore[attr-defined]
    rclpy_mod.node = node_mod  # type: ignore[attr-defined]
    rclpy_mod.qos = qos_mod  # type: ignore[attr-defined]
    rclpy_mod.time = time_mod  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.duration', duration_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.node', node_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.qos', qos_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.time', time_mod)

    _stub_msg_package(monkeypatch, 'geometry_msgs', ['Point', 'Vector3'])
    _stub_msg_package(monkeypatch, 'nav_msgs', ['Odometry'])
    _stub_msg_package(monkeypatch, 'std_msgs', ['Bool', 'ColorRGBA', 'String'])
    _stub_msg_package(monkeypatch, 'visualization_msgs', ['Marker'])
    _stub_msg_package(monkeypatch, 'gazebo_target_sim_interfaces', ['ImpactEvent'])
    _stub_msg_package(monkeypatch, 'rosgraph_msgs', ['Clock'])

    path = _PKG_ROOT / 'gazebo_target_sim' / 'interception_logic_node.py'
    spec = importlib.util.spec_from_file_location('interception_logic_node_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_kinematic_rollout_honors_initial_interceptor_velocity(monkeypatch) -> None:  # noqa: ANN001
    mod = _load_interception_module(monkeypatch)

    kwargs = dict(
        v_i_max=80.0,
        t_min=0.0,
        t_max=0.3,
        hit_thresh_m=5.0,
        pos_sigma_m=0.0,
        vel_sigma_m_s=0.0,
        interceptor_pos_sigma_m=0.0,
        delay_mean_s=0.0,
        delay_jitter_s=0.0,
        rng=random.Random(1),
        use_kinematic_rollout=True,
        rollout_dt=0.05,
        rollout_max_turn_rate_rad_s=0.4,
        rollout_max_accel_m_s2=5.0,
    )

    starts_from_rest = mod.simulate_intercept_once(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, **kwargs)
    already_closing = mod.simulate_intercept_once(
        10.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        interceptor_initial_velocity=(10.0, 0.0, 0.0),
        **kwargs,
    )

    assert starts_from_rest is False
    assert already_closing is True


def test_rollout_gate_forwards_current_velocity(monkeypatch) -> None:  # noqa: ANN001
    mod = _load_interception_module(monkeypatch)
    node = object.__new__(mod.InterceptionLogicNode)
    node._eng_rollout_gate_horizon_param = 0.0
    node._t_hit_max = 1.0
    node._interceptor_max_speed = 80.0
    node._t_hit_min = 0.0
    node._hit_thresh = 1.0
    node._heatmap_prob_rollout_dt = 0.05
    node._max_turn_rate = 0.4
    node._max_accel = 5.0
    node._heatmap_prob_rollout_tau = 0.0
    node._heatmap_prob_rollout_cmd_delay = 0.0

    captured = {}

    def fake_simulate(*_args, **kwargs):  # noqa: ANN002, ANN003, ANN202
        captured.update(kwargs)
        return True

    monkeypatch.setattr(mod, 'simulate_intercept_once', fake_simulate)

    assert node._eng_rollout_gate_passes(
        10.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        interceptor_initial_velocity=(10.0, 0.0, 0.0),
    )
    assert captured['interceptor_initial_velocity'] == (10.0, 0.0, 0.0)
