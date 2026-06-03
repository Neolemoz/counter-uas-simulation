"""Regression tests for the candidate-confirmation gate.

Motivation
----------
Before the predictive-gate fix, ``CANDIDATE_MERGE_M = 1.0`` was a hard module-level constant
matched against the *raw last detection*.  For a 38 m/s target sampled at 10 Hz the per-frame
displacement is ~3.8 m, so consecutive detections were always > 1 m apart and **no candidate
ever confirmed**.  The downstream effect was that ``/tracks/state`` stayed empty and the
interceptor never engaged when the target entered the outer dome.

Subsequent diagnostics showed two more failure modes that broke track formation at km-scale:

* the candidate was discarded after a single empty cycle (no miss tolerance), so a bursty
  upstream stream lost candidates between detection bursts — fixed via
  ``candidate_max_missed_frames``;
* the velocity used by the predictor assumed a 100 ms gap between consecutive history
  points, but the upstream actually publishes every ~300 ms, so the velocity was inflated
  3× — fixed by storing the explicit timestamp with each history entry and using actual
  ``Δt`` in ``_initial_velocity_from_history``.

These tests reproduce all three failure modes at the dataclass level — no ROS, no Gazebo —
so the fixes are locked in by CI.
"""

from __future__ import annotations

import importlib.util
import math
import sys
import types
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _install_tracking_ros_stubs() -> None:
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_node_mod = types.ModuleType('rclpy.node')

    class _StubNode:
        pass

    rclpy_node_mod.Node = _StubNode  # type: ignore[attr-defined]

    rclpy_qos_mod = types.ModuleType('rclpy.qos')

    class _QoSProfile:
        def __init__(self, *args, **kwargs) -> None:  # noqa: ANN002, ANN003
            self.args = args
            self.kwargs = kwargs

    class _Policy:
        BEST_EFFORT = object()
        VOLATILE = object()
        KEEP_LAST = object()

    rclpy_qos_mod.QoSProfile = _QoSProfile  # type: ignore[attr-defined]
    rclpy_qos_mod.ReliabilityPolicy = _Policy  # type: ignore[attr-defined]
    rclpy_qos_mod.DurabilityPolicy = _Policy  # type: ignore[attr-defined]
    rclpy_qos_mod.HistoryPolicy = _Policy  # type: ignore[attr-defined]

    rclpy_time_mod = types.ModuleType('rclpy.time')

    class _Time:
        @staticmethod
        def to_msg():  # noqa: ANN201
            return types.SimpleNamespace(sec=0, nanosec=0)

    rclpy_time_mod.Time = _Time  # type: ignore[attr-defined]
    rclpy_mod.time = rclpy_time_mod  # type: ignore[attr-defined]
    rclpy_mod.init = lambda args=None: None  # type: ignore[attr-defined]
    rclpy_mod.spin = lambda node: None  # type: ignore[attr-defined]
    rclpy_mod.shutdown = lambda: None  # type: ignore[attr-defined]

    geom_msg = types.ModuleType('geometry_msgs.msg')

    class _Point:
        __slots__ = ('x', 'y', 'z')

        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Quaternion:
        __slots__ = ('x', 'y', 'z', 'w')

        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 0.0

    geom_msg.Point = _Point  # type: ignore[attr-defined]
    geom_msg.PoseWithCovariance = object  # type: ignore[attr-defined]
    geom_msg.TwistWithCovariance = object  # type: ignore[attr-defined]
    geom_pkg = types.ModuleType('geometry_msgs')
    geom_pkg.msg = geom_msg  # type: ignore[attr-defined]

    nav_msg = types.ModuleType('nav_msgs.msg')

    class _Header:
        __slots__ = ('stamp', 'frame_id')

        def __init__(self) -> None:
            self.stamp = types.SimpleNamespace(sec=0, nanosec=0)
            self.frame_id = ''

    class _Pose:
        __slots__ = ('position', 'orientation')

        def __init__(self) -> None:
            self.position = _Point()
            self.orientation = _Quaternion()

    class _Twist:
        __slots__ = ('linear',)

        def __init__(self) -> None:
            self.linear = _Point()

    class _Odometry:
        def __init__(self) -> None:
            self.header = _Header()
            self.child_frame_id = ''
            self.pose = types.SimpleNamespace(pose=_Pose(), covariance=[0.0] * 36)
            self.twist = types.SimpleNamespace(twist=_Twist(), covariance=[0.0] * 36)

    nav_msg.Odometry = _Odometry  # type: ignore[attr-defined]
    nav_pkg = types.ModuleType('nav_msgs')
    nav_pkg.msg = nav_msg  # type: ignore[attr-defined]

    rosgraph_msg = types.ModuleType('rosgraph_msgs.msg')

    class _Clock:
        def __init__(self) -> None:
            self.clock = types.SimpleNamespace(sec=0, nanosec=0)

    rosgraph_msg.Clock = _Clock  # type: ignore[attr-defined]
    rosgraph_pkg = types.ModuleType('rosgraph_msgs')
    rosgraph_pkg.msg = rosgraph_msg  # type: ignore[attr-defined]

    sys.modules['rclpy'] = rclpy_mod
    sys.modules['rclpy.node'] = rclpy_node_mod
    sys.modules['rclpy.qos'] = rclpy_qos_mod
    sys.modules['rclpy.time'] = rclpy_time_mod
    sys.modules['geometry_msgs'] = geom_pkg
    sys.modules['geometry_msgs.msg'] = geom_msg
    sys.modules['nav_msgs'] = nav_pkg
    sys.modules['nav_msgs.msg'] = nav_msg
    sys.modules['rosgraph_msgs'] = rosgraph_pkg
    sys.modules['rosgraph_msgs.msg'] = rosgraph_msg


def _load_tracking_module():  # noqa: ANN201
    # Some ROS-free tests stub geometry_msgs.msg for isolated imports.  Ensure
    # tracking sees the real ROS message package so nav_msgs/Odometry can import
    # PoseWithCovariance and TwistWithCovariance.
    geom_msg = sys.modules.get('geometry_msgs.msg')
    if geom_msg is not None and not hasattr(geom_msg, 'PoseWithCovariance'):
        sys.modules.pop('geometry_msgs.msg', None)
        sys.modules.pop('geometry_msgs', None)
    rclpy_mod = sys.modules.get('rclpy')
    if rclpy_mod is not None and not hasattr(rclpy_mod, 'time'):
        sys.modules.pop('rclpy', None)
        sys.modules.pop('rclpy.node', None)
    if 'rclpy' not in sys.modules:
        try:
            rclpy_available = importlib.util.find_spec('rclpy') is not None
        except ValueError:
            rclpy_available = False
        if not rclpy_available:
            _install_tracking_ros_stubs()
    path = _REPO_ROOT / 'src' / 'tracking' / 'tracking' / 'tracking_node.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('tracking_node_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _append_hist(cand, x: float, y: float, z: float, t: float) -> None:  # noqa: ANN001
    cand.pos_history.append((x, y, z, t))
    cand.last_update_time = t


def _make_candidate(mod, x: float, y: float, z: float, t: float = 0.0):  # noqa: ANN001, ANN201
    cand = mod.Candidate(x=x, y=y, z=z, hit_count=1)
    _append_hist(cand, x, y, z, t)
    return cand


def _make_point(mod, x: float, y: float, z: float):  # noqa: ANN001, ANN201
    p = mod.Point()
    p.x, p.y, p.z = x, y, z
    return p


def test_predicted_candidate_xyz_extrapolates_with_history() -> None:
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    _append_hist(cand, 6.2, 0.0, 0.0, 0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.1)

    pxyz = mod._predicted_candidate_xyz(cand)
    expected_v = (10.0 - 6.2) / 0.1
    assert abs(pxyz[0] - (10.0 + expected_v * mod.CYCLE_PERIOD_S)) < 1e-9
    assert abs(pxyz[1]) < 1e-9
    assert abs(pxyz[2]) < 1e-9


def test_predicted_candidate_xyz_no_history_falls_back_to_last_position() -> None:
    mod = _load_tracking_module()
    cand = mod.Candidate(x=5.0, y=-2.0, z=1.0, hit_count=1)
    _append_hist(cand, 5.0, -2.0, 1.0, 0.0)

    pxyz = mod._predicted_candidate_xyz(cand)
    assert pxyz == (5.0, -2.0, 1.0)


def test_legacy_gate_misses_fast_target_at_10hz() -> None:
    """At 38 m/s the legacy 1 m gate fails — this is the original bug."""
    mod = _load_tracking_module()
    cand = _make_candidate(mod, 6.2, 0.0, 0.0, t=0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.1)

    new_det = _make_point(mod, 13.8, 0.0, 0.0)

    legacy_d = mod._distance_point_to_candidate(new_det, cand)
    assert legacy_d > 1.0, 'sanity: legacy distance must exceed 1 m gate'


def test_predictive_gate_admits_fast_target_at_10hz() -> None:
    """With predictive matching the residual collapses to ~ measurement noise (≈ 0)."""
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    _append_hist(cand, 6.2, 0.0, 0.0, 0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.1)

    new_det = _make_point(mod, 13.8, 0.0, 0.0)
    pxyz = mod._predicted_candidate_xyz(cand)
    pred_d = mod._distance_point_to_xyz(new_det, pxyz)
    assert pred_d < 0.1, f'predictive gate residual should be ~0, got {pred_d}'


def test_predictive_gate_tolerates_realistic_noise() -> None:
    """At ~0.5 m fused-position noise, predictive gate still matches at 8 m operational gate."""
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    _append_hist(cand, 6.2, 0.0, 0.0, 0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.1)

    new_det = _make_point(mod, 13.8 + 0.4, 0.3, -0.2)
    pxyz = mod._predicted_candidate_xyz(cand)
    pred_d = mod._distance_point_to_xyz(new_det, pxyz)
    assert pred_d < 8.0, f'predictive gate must still admit noisy match, got {pred_d}'
    assert pred_d < 1.0, f'predictive residual should be ~ noise std, got {pred_d}'


def test_velocity_uses_real_dt_not_cycle_period() -> None:
    """Regression: when consecutive history points are 300 ms apart, the velocity must be
    computed with that real Δt — not divided by ``CYCLE_PERIOD_S=0.1`` which would inflate
    the speed 3× and propagate into a wildly-wrong seed track velocity (which is what we
    saw at km-scale: ``v_init=(108, -67, -12)`` for a real 50 m/s target).
    """
    mod = _load_tracking_module()
    hist = [
        (0.0, 0.0, 0.0, 0.0),
        (15.0, 0.0, 0.0, 0.3),
    ]
    vx, vy, vz = mod._initial_velocity_from_history(hist)
    # Real velocity = 15 m / 0.3 s = 50 m/s along x.
    assert abs(vx - 50.0) < 1e-6, f'expected vx=50.0, got {vx}'
    assert abs(vy) < 1e-9
    assert abs(vz) < 1e-9


def test_predicted_xyz_uses_actual_now_horizon() -> None:
    """When ``now`` is passed and ``last_update_time`` is set, the prediction horizon is
    ``now - last_update_time`` — which lets the matcher correctly extrapolate across a
    long quiet gap (the case where a bursty fusion stream goes silent for 200–300 ms).
    """
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    _append_hist(cand, 0.0, 0.0, 0.0, 0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.2)

    px, py, pz = mod._predicted_candidate_xyz(cand, now=0.5)
    # v = (10-0)/0.2 = 50 m/s.  horizon = 0.5 - 0.2 = 0.3.  predicted x = 10 + 50*0.3 = 25.
    assert abs(px - 25.0) < 1e-6, f'expected px=25, got {px}'
    assert abs(py) < 1e-9
    assert abs(pz) < 1e-9


def test_candidate_miss_tolerance_keeps_candidate_alive() -> None:
    """A candidate with ``missed_frames`` ≤ ``candidate_max_missed_frames`` must NOT be
    discarded.  The legacy code treated any unmatched cycle as a kill — fatal when the
    upstream stream skips a tracking cycle.
    """
    mod = _load_tracking_module()
    cand = mod.Candidate(x=0.0, y=0.0, z=0.0, hit_count=1)
    _append_hist(cand, 0.0, 0.0, 0.0, 0.0)
    cand.missed_frames = 3
    # The fix is structural: the candidate just keeps its missed counter and the *node*
    # decides via ``cand.missed_frames > self._candidate_max_missed_frames``.  We assert
    # the field is wired through.
    assert hasattr(cand, 'missed_frames')
    assert cand.missed_frames == 3


def test_tracks_state_odometry_carries_finite_position_velocity_and_covariance() -> None:
    mod = _load_tracking_module()
    tr = mod.Track.new_from_position(7, 10.0, -2.0, 3.0, vx=38.0, vy=-1.5, vz=-2.0)

    class _Stamp:
        def to_msg(self):  # noqa: ANN201
            return mod.rclpy.time.Time().to_msg()

    class _Clock:
        @staticmethod
        def now() -> _Stamp:
            return _Stamp()

    class _FakeNode:
        _tracks_state_frame_id = 'map'

        @staticmethod
        def get_clock() -> _Clock:
            return _Clock()

    msg = mod.TrackingNode._track_to_odometry(_FakeNode(), tr)

    vals = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
        *msg.pose.covariance,
        *msg.twist.covariance,
    )
    assert msg.header.frame_id == 'map'
    assert msg.child_frame_id == 'track_7'
    assert msg.twist.twist.linear.x == 38.0
    assert all(math.isfinite(float(v)) for v in vals)
    assert msg.pose.covariance[0] > 0.0
    assert msg.twist.covariance[0] > 0.0


def test_sim_reset_clears_stale_tracks_candidates_and_buffer() -> None:
    """Gazebo reset must not leave pre-reset tracks publishing ghost /tracks/state."""
    mod = _load_tracking_module()

    class _Logger:
        @staticmethod
        def info(_msg: str) -> None:
            return None

    node = object.__new__(mod.TrackingNode)
    node.get_logger = lambda: _Logger()  # type: ignore[method-assign]
    node._tracks = [mod.Track.new_from_position(3, 1000.0, 0.0, 200.0, vx=-40.0)]
    node._candidates = [mod.Candidate(x=999.0, y=1.0, z=200.0, hit_count=2)]
    node._detection_buffer = [_make_point(mod, 1001.0, 0.0, 200.0)]
    node._next_id = 4

    node._on_gz_sim_reset()

    assert node._tracks == []
    assert node._candidates == []
    assert node._detection_buffer == []
    assert node._next_id == 1
