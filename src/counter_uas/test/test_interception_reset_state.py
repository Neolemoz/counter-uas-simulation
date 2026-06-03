"""Regression tests for Gazebo reset cleanup in interception logic."""

from __future__ import annotations

import importlib.util
import sys
import types
from collections import deque
from pathlib import Path
from unittest.mock import MagicMock

_REPO_ROOT = Path(__file__).resolve().parents[3]
_GAZEBO_PKG_ROOT = _REPO_ROOT / 'src' / 'gazebo_target_sim'


def _install_ros_stubs(monkeypatch):  # noqa: ANN001, ANN202
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_mod.init = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.shutdown = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.spin = MagicMock()  # type: ignore[attr-defined]

    rclpy_node_mod = types.ModuleType('rclpy.node')

    class _StubNode:
        def get_logger(self):  # noqa: ANN201
            log = MagicMock()
            log.info = MagicMock()
            log.warning = MagicMock()
            return log

        def destroy_timer(self, timer):  # noqa: ANN001
            self._destroyed_timers.append(timer)

    rclpy_node_mod.Node = _StubNode  # type: ignore[attr-defined]

    rclpy_duration_mod = types.ModuleType('rclpy.duration')

    class _Duration:
        def __init__(self, seconds: float = 0.0) -> None:
            self.nanoseconds = int(seconds * 1_000_000_000)

    rclpy_duration_mod.Duration = _Duration  # type: ignore[attr-defined]

    rclpy_time_mod = types.ModuleType('rclpy.time')

    class _Time:
        nanoseconds = 0

        def __sub__(self, _other):  # noqa: ANN001, ANN201
            return _Duration(0.0)

        def __add__(self, other):  # noqa: ANN001, ANN201
            out = _Time()
            out.nanoseconds = self.nanoseconds + getattr(other, 'nanoseconds', 0)
            return out

        @staticmethod
        def to_msg():  # noqa: ANN201
            return types.SimpleNamespace(sec=0, nanosec=0)

    rclpy_time_mod.Time = _Time  # type: ignore[attr-defined]
    rclpy_mod.time = rclpy_time_mod  # type: ignore[attr-defined]

    rclpy_qos_mod = types.ModuleType('rclpy.qos')

    class _QoSProfile:
        def __init__(self, *args, **kwargs) -> None:  # noqa: ANN002, ANN003
            self.args = args
            self.kwargs = kwargs

    class _Policy:
        BEST_EFFORT = object()
        RELIABLE = object()
        VOLATILE = object()
        TRANSIENT_LOCAL = object()
        KEEP_LAST = object()

    rclpy_qos_mod.QoSProfile = _QoSProfile  # type: ignore[attr-defined]
    rclpy_qos_mod.ReliabilityPolicy = _Policy  # type: ignore[attr-defined]
    rclpy_qos_mod.DurabilityPolicy = _Policy  # type: ignore[attr-defined]
    rclpy_qos_mod.HistoryPolicy = _Policy  # type: ignore[attr-defined]

    geom_msg_mod = types.ModuleType('geometry_msgs.msg')

    class _Point:
        __slots__ = ('x', 'y', 'z')

        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
            self.x = x
            self.y = y
            self.z = z

    class _Vector3(_Point):
        pass

    geom_msg_mod.Point = _Point  # type: ignore[attr-defined]
    geom_msg_mod.Vector3 = _Vector3  # type: ignore[attr-defined]
    geom_pkg = types.ModuleType('geometry_msgs')
    geom_pkg.msg = geom_msg_mod  # type: ignore[attr-defined]

    nav_msg_mod = types.ModuleType('nav_msgs.msg')

    class _Odometry:
        def __init__(self) -> None:
            self.pose = types.SimpleNamespace(
                pose=types.SimpleNamespace(position=_Point()),
            )
            self.twist = types.SimpleNamespace(
                twist=types.SimpleNamespace(linear=_Vector3()),
            )

    nav_msg_mod.Odometry = _Odometry  # type: ignore[attr-defined]
    nav_pkg = types.ModuleType('nav_msgs')
    nav_pkg.msg = nav_msg_mod  # type: ignore[attr-defined]

    std_msg_mod = types.ModuleType('std_msgs.msg')

    class _Bool:
        def __init__(self, data: bool = False) -> None:
            self.data = data

    class _ColorRGBA:
        def __init__(self, r: float = 0.0, g: float = 0.0, b: float = 0.0, a: float = 0.0) -> None:
            self.r = r
            self.g = g
            self.b = b
            self.a = a

    class _String:
        def __init__(self, data: str = '') -> None:
            self.data = data

    std_msg_mod.Bool = _Bool  # type: ignore[attr-defined]
    std_msg_mod.ColorRGBA = _ColorRGBA  # type: ignore[attr-defined]
    std_msg_mod.String = _String  # type: ignore[attr-defined]
    std_pkg = types.ModuleType('std_msgs')
    std_pkg.msg = std_msg_mod  # type: ignore[attr-defined]

    marker_msg_mod = types.ModuleType('visualization_msgs.msg')

    class _Marker:
        DELETEALL = 3

        def __init__(self) -> None:
            self.header = types.SimpleNamespace(frame_id='', stamp=None)
            self.ns = ''
            self.action = 0

    marker_msg_mod.Marker = _Marker  # type: ignore[attr-defined]
    marker_pkg = types.ModuleType('visualization_msgs')
    marker_pkg.msg = marker_msg_mod  # type: ignore[attr-defined]

    rosgraph_msg_mod = types.ModuleType('rosgraph_msgs.msg')

    class _Clock:
        def __init__(self) -> None:
            self.clock = types.SimpleNamespace(sec=0, nanosec=0)

    rosgraph_msg_mod.Clock = _Clock  # type: ignore[attr-defined]
    rosgraph_pkg = types.ModuleType('rosgraph_msgs')
    rosgraph_pkg.msg = rosgraph_msg_mod  # type: ignore[attr-defined]

    impact_msg_mod = types.ModuleType('gazebo_target_sim_interfaces.msg')

    class _ImpactEvent:
        def __init__(self) -> None:
            self.interceptor_id = ''
            self.target_label = ''
            self.stamp = None

    impact_msg_mod.ImpactEvent = _ImpactEvent  # type: ignore[attr-defined]
    impact_pkg = types.ModuleType('gazebo_target_sim_interfaces')
    impact_pkg.msg = impact_msg_mod  # type: ignore[attr-defined]

    for name, module in {
        'rclpy': rclpy_mod,
        'rclpy.node': rclpy_node_mod,
        'rclpy.duration': rclpy_duration_mod,
        'rclpy.time': rclpy_time_mod,
        'rclpy.qos': rclpy_qos_mod,
        'geometry_msgs': geom_pkg,
        'geometry_msgs.msg': geom_msg_mod,
        'nav_msgs': nav_pkg,
        'nav_msgs.msg': nav_msg_mod,
        'std_msgs': std_pkg,
        'std_msgs.msg': std_msg_mod,
        'visualization_msgs': marker_pkg,
        'visualization_msgs.msg': marker_msg_mod,
        'rosgraph_msgs': rosgraph_pkg,
        'rosgraph_msgs.msg': rosgraph_msg_mod,
        'gazebo_target_sim_interfaces': impact_pkg,
        'gazebo_target_sim_interfaces.msg': impact_msg_mod,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    monkeypatch.syspath_prepend(str(_GAZEBO_PKG_ROOT))


def _load_interception_module(monkeypatch):  # noqa: ANN001, ANN201
    _install_ros_stubs(monkeypatch)
    path = _GAZEBO_PKG_ROOT / 'gazebo_target_sim' / 'interception_logic_node.py'
    spec = importlib.util.spec_from_file_location('interception_logic_node_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _make_reset_node(mod):  # noqa: ANN001, ANN201
    node = object.__new__(mod.InterceptionLogicNode)
    node._destroyed_timers = []
    node._ids = ['interceptor_0', 'interceptor_1']
    node._hit = True
    node._gz_pause_sent = True
    node._dome_hyst_initialized = True
    node._pause_world_timer = object()
    node._stop_repeat_timer = object()
    node._stop_repeat_deadline = object()
    node._stop_repeat_pending_label = 'target_0'
    node._target = mod.Point(1200.0, -40.0, 200.0)
    node._target_filter_velocity = (-40.0, 0.0, -5.0)
    node._locked_selected_id = 'interceptor_0'
    node._current_selected_id = 'interceptor_0'
    node._committed_since = object()
    node._lost_since = object()
    node._reacquire_since = object()
    node._switch_count = 3
    node._best_id_last = 'interceptor_0'
    node._assigned_interceptor_id = 'interceptor_0'
    node._assignment_time = object()
    node._last_assign_lock_log = object()
    node._mc_engage_state = {'interceptor_0': object()}
    node._prev_velocity = {iid: (1.0, 2.0, 3.0) for iid in node._ids}
    node._last_control_time = object()
    node._control_dt_default = 0.1
    node._control_dt = 0.4
    node._intercept_point_filtered = {iid: (1.0, 2.0, 3.0) for iid in node._ids}
    node._guidance_mode = {iid: 'predict' for iid in node._ids}
    node._valid_streak = {iid: 5 for iid in node._ids}
    node._invalid_streak = {iid: 2 for iid in node._ids}
    node._t_go_filtered = {iid: 7.5 for iid in node._ids}
    node._guidance_unit_prev = {iid: (0.0, 1.0, 0.0) for iid in node._ids}
    node._last_layer = 'engage'
    node._last_feas_log = object()
    node._last_class_warn = object()
    node._v_tgt_smooth = (-40.0, 0.0, -5.0)
    node._prev_target = (1200.0, -40.0, 200.0)
    node._prev_target_time = object()
    node._hit_snap_target_prev = (1200.0, -40.0, 200.0)
    node._hit_snap_target_prev_multi = {'target_0': (1.0, 2.0, 3.0)}
    node._target_detect_time = object()
    node._prev_inter_pos = {iid: (1.0, 2.0, 3.0) for iid in node._ids}
    node._prev_inter_time = {iid: object() for iid in node._ids}
    node._multi_enabled = False
    node._multi_labels = []
    node._min_miss_distance = 12.0
    node._last_hit_range = {iid: 2.0 for iid in node._ids}
    node._logged_guard_block = True
    node._trail_target = deque([(1.0, 2.0, 3.0)])
    node._trail_inters = {iid: deque([(1.0, 2.0, 3.0)]) for iid in node._ids}
    node._trail_multi_hostile = {}
    node._math_trail_target = deque([(1.0, 2.0, 3.0)])
    node._math_trail_inters = {iid: deque([(1.0, 2.0, 3.0)]) for iid in node._ids}
    node._feasible_at_engagement_start_by_pair = {('', 'interceptor_0'): True}
    node._feas_eng_latch_assign = {'interceptor_0': 'target_0'}
    node._last_feas_debug_log = object()
    node._last_feas_warn_log = object()
    node._pub_intercept_viz = None
    node._pub_heatmap_prob = None
    node._heatmap_prob_cache = {'x': 1}
    node._mc_p_last = {'interceptor_0': 0.9}
    node._heatmap_prob_skip_next = True
    node._last_guidance_cmd = {iid: (1.0, 0.0, 0.0) for iid in node._ids}
    node._mc_high_p_start_m = {'interceptor_0': 1.0}
    node._last_phase1_high_p_warn = {'interceptor_0': 1.0}
    node._last_phase1_vreq_warn_m = 1.0
    node._last_phase1_engage_warn_m = 1.0
    node._dbg_no_hit_watch_start_m = {'interceptor_0': 1.0}
    node._dbg_no_hit_watch_tgo = {'interceptor_0': 1.0}
    node._dbg_no_hit_warn_last_m = {'interceptor_0': 1.0}
    node._last_hit_debug_print_m = 1.0
    node._last_hit_gate_block_print_m = 1.0
    node._multi_hit_safety_logged = {('target_0', 'interceptor_0')}
    return node


def test_interception_reset_clears_single_target_guidance_and_pause_timer(monkeypatch) -> None:  # noqa: ANN001
    mod = _load_interception_module(monkeypatch)
    node = _make_reset_node(mod)
    pause_timer = node._pause_world_timer
    stop_timer = node._stop_repeat_timer

    node._on_gz_sim_reset()

    assert node._hit is False
    assert node._gz_pause_sent is False
    assert node._target is None
    assert node._target_filter_velocity is None
    assert node._prev_target is None
    assert node._target_detect_time is None
    assert node._pause_world_timer is None
    assert node._stop_repeat_timer is None
    assert pause_timer in node._destroyed_timers
    assert stop_timer in node._destroyed_timers
    assert node._locked_selected_id is None
    assert node._current_selected_id is None
    assert node._assigned_interceptor_id is None
    assert node._mc_engage_state == {}
    assert node._feasible_at_engagement_start_by_pair == {}
    assert node._feas_eng_latch_assign == {}
    for iid in node._ids:
        assert node._intercept_point_filtered[iid] is None
        assert node._guidance_mode[iid] == 'pursuit'
        assert node._valid_streak[iid] == 0
        assert node._invalid_streak[iid] == 0
        assert node._t_go_filtered[iid] is None
        assert iid not in node._guidance_unit_prev
        assert node._prev_velocity[iid] == (0.0, 0.0, 0.0)
        assert node._prev_inter_pos[iid] is None
        assert node._prev_inter_time[iid] is None
        assert node._last_guidance_cmd[iid] == (0.0, 0.0, 0.0)


def test_pause_world_timer_is_one_shot(monkeypatch) -> None:  # noqa: ANN001
    mod = _load_interception_module(monkeypatch)
    node = object.__new__(mod.InterceptionLogicNode)
    node._destroyed_timers = []
    node._pause_world_timer = object()
    timer = node._pause_world_timer
    node._gz_pause_sent = False
    node._pause_gz_on_hit = False

    node._pause_gazebo_world()

    assert node._pause_world_timer is None
    assert timer in node._destroyed_timers
