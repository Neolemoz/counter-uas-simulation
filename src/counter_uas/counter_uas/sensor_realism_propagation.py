"""Headless propagation harness: GT callbacks → sensors → fusion → tracking."""

from __future__ import annotations

import importlib.util
import math
import random
import sys
import types
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

_REPO_ROOT = Path(__file__).resolve().parents[3]


@dataclass
class SensorTimingConfig:
    publish_every_n: int = 1
    delay_mean_s: float = 0.0
    delay_jitter_s: float = 0.0
    seed: int = -1


@dataclass
class SensorRangeConfig:
    max_range_m: float = 12.0
    base_p_detect: float = 1.0
    pd_decay: float = 0.0
    pd_min: float = 0.0
    std_xy: float = 0.5
    std_z: float = 0.2
    std_scale_with_range: float = 0.0


@dataclass
class PropagationCounts:
    gt_callbacks: int = 0
    decimated_callbacks: int = 0
    delayed_callbacks: int = 0
    radar_published: int = 0
    camera_published: int = 0
    fused_published: int = 0
    tracks_state_published: int = 0
    mean_fused_error_near_m: float = 0.0
    mean_fused_error_far_m: float = 0.0


@dataclass
class PropagationProfile:
    name: str
    radar_timing: SensorTimingConfig
    camera_timing: SensorTimingConfig
    radar_range: SensorRangeConfig
    camera_range: SensorRangeConfig


def _load_module(relative: str, module_name: str):  # noqa: ANN201
    path = _REPO_ROOT / relative
    spec = importlib.util.spec_from_file_location(module_name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod


def _load_timing():  # noqa: ANN201
    return _load_module('src/radar_sim/radar_sim/timing_realism.py', 'timing_realism_harness')


def _load_range():  # noqa: ANN201
    return _load_module('src/radar_sim/radar_sim/range_realism.py', 'range_realism_harness')


def _stub_ros_for_fusion_and_tracking() -> Any:
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_mod.init = MagicMock()
    rclpy_mod.shutdown = MagicMock()
    rclpy_mod.spin = MagicMock()
    rclpy_node_mod = types.ModuleType('rclpy.node')

    class _Node:
        def __init__(self, name: str = '') -> None:
            self._name = name
            self._params: dict[str, Any] = {}
            self._clock = MagicMock()
            self._clock.now.return_value.nanoseconds = 0

        def declare_parameter(self, name: str, value: Any = None) -> MagicMock:
            self._params[name] = value
            param = MagicMock()
            param.value = value
            return param

        def get_parameter(self, name: str) -> MagicMock:
            param = MagicMock()
            param.value = self._params.get(name)
            return param

        def get_logger(self) -> MagicMock:
            return MagicMock()

        def create_publisher(self, *args: Any, **kwargs: Any) -> MagicMock:
            return MagicMock()

        def create_subscription(self, *args: Any, **kwargs: Any) -> MagicMock:
            return MagicMock()

        def create_timer(self, *args: Any, **kwargs: Any) -> MagicMock:
            return MagicMock()

        def get_clock(self) -> MagicMock:
            return self._clock

    rclpy_node_mod.Node = _Node
    geom_pkg = types.ModuleType('geometry_msgs')
    geom_mod = types.ModuleType('geometry_msgs.msg')

    class Point:
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    geom_mod.Point = Point
    nav_pkg = types.ModuleType('nav_msgs')
    nav_mod = types.ModuleType('nav_msgs.msg')

    class Odometry:
        def __init__(self) -> None:
            self.header = MagicMock()
            self.child_frame_id = ''
            self.pose = MagicMock()
            self.twist = MagicMock()

    nav_mod.Odometry = Odometry

    sys.modules.setdefault('rclpy', rclpy_mod)
    sys.modules.setdefault('rclpy.node', rclpy_node_mod)
    sys.modules.setdefault('geometry_msgs', geom_pkg)
    sys.modules.setdefault('geometry_msgs.msg', geom_mod)
    sys.modules.setdefault('nav_msgs', nav_pkg)
    sys.modules.setdefault('nav_msgs.msg', nav_mod)
    return geom_mod


def _load_fusion_module():  # noqa: ANN201
    _stub_ros_for_fusion_and_tracking()
    module_name = 'fusion.fusion_node'
    if module_name in sys.modules:
        return sys.modules[module_name]
    tracking_pkg = types.ModuleType('fusion')
    sys.modules['fusion'] = tracking_pkg
    return _load_module('src/fusion/fusion/fusion_node.py', module_name)


def _load_tracking_module():  # noqa: ANN201
    geom_mod = _stub_ros_for_fusion_and_tracking()
    module_name = 'tracking.tracking_node'
    if module_name in sys.modules:
        return sys.modules[module_name], geom_mod
    tracking_pkg = types.ModuleType('tracking')
    tracking_sub = types.ModuleType('tracking.tracking')
    sys.modules['tracking'] = tracking_pkg
    sys.modules['tracking.tracking'] = tracking_sub
    mod = _load_module('src/tracking/tracking/tracking_node.py', module_name)
    return mod, geom_mod


def baseline_profile() -> PropagationProfile:
    return PropagationProfile(
        name='baseline',
        radar_timing=SensorTimingConfig(),
        camera_timing=SensorTimingConfig(),
        radar_range=SensorRangeConfig(),
        camera_range=SensorRangeConfig(max_range_m=20.0, std_xy=0.2, std_z=0.1),
    )


def overlay_profile_from_docs() -> PropagationProfile:
    from counter_uas.sensor_realism_overlays import load_combined_sensor_overrides

    merged = load_combined_sensor_overrides()
    radar = merged.get('radar_sim_node', {}).get('radar', {})
    camera = merged.get('camera_sim_node', {}).get('camera', {})

    return PropagationProfile(
        name='sensor_realism_overlay',
        radar_timing=SensorTimingConfig(
            publish_every_n=int(radar.get('publish_every_n', 2)),
            delay_mean_s=float(radar.get('delay_mean_s', 0.0)),
            delay_jitter_s=float(radar.get('delay_jitter_s', 0.0)),
            seed=int(radar.get('seed', 42)),
        ),
        camera_timing=SensorTimingConfig(
            publish_every_n=int(camera.get('publish_every_n', 3)),
            delay_mean_s=float(camera.get('delay_mean_s', 0.0)),
            delay_jitter_s=float(camera.get('delay_jitter_s', 0.0)),
            seed=int(camera.get('seed', 43)),
        ),
        radar_range=SensorRangeConfig(
            pd_decay=float(radar.get('detection_probability_decay_with_range', 0.0)),
            pd_min=float(radar.get('min_detection_probability', 0.0)),
            std_scale_with_range=float(radar.get('measurement_std_scale_with_range', 0.0)),
        ),
        camera_range=SensorRangeConfig(
            max_range_m=20.0,
            base_p_detect=0.98,
            pd_decay=float(camera.get('detection_probability_decay_with_range', 0.0)),
            pd_min=float(camera.get('min_detection_probability', 0.0)),
            std_xy=0.2,
            std_z=0.1,
            std_scale_with_range=float(camera.get('measurement_std_scale_with_range', 0.0)),
        ),
    )


def _make_point(geom_mod: Any, x: float, y: float, z: float) -> Any:
    p = geom_mod.Point()
    p.x, p.y, p.z = x, y, z
    return p


def _sensor_publish(
    *,
    callback_index: int,
    truth: tuple[float, float, float],
    timing: SensorTimingConfig,
    rng_cfg: SensorRangeConfig,
    timing_mod: Any,
    range_mod: Any,
    rng: random.Random,
) -> tuple[tuple[float, float, float] | None, int, int]:
    if not timing_mod.should_publish_on_callback(callback_index, timing.publish_every_n):
        return None, 1, 0

    distance = math.sqrt(truth[0] ** 2 + truth[1] ** 2 + truth[2] ** 2)
    if distance > rng_cfg.max_range_m:
        return None, 0, 0

    p_eff = range_mod.effective_detection_probability(
        base_p=rng_cfg.base_p_detect,
        distance_m=distance,
        max_range_m=rng_cfg.max_range_m,
        decay_with_range=rng_cfg.pd_decay,
        min_detection_probability=rng_cfg.pd_min,
    )
    if rng.random() > p_eff:
        return None, 0, 0

    std_xy = range_mod.effective_measurement_std(
        rng_cfg.std_xy, distance, rng_cfg.max_range_m, rng_cfg.std_scale_with_range
    )
    std_z = range_mod.effective_measurement_std(
        rng_cfg.std_z, distance, rng_cfg.max_range_m, rng_cfg.std_scale_with_range
    )
    delay_s = timing_mod.transport_delay_s(
        timing.delay_mean_s, timing.delay_jitter_s, rng if timing.seed >= 0 else None
    )
    delayed = 1 if delay_s > 1e-4 else 0
    noisy = (
        truth[0] + rng.gauss(0.0, std_xy),
        truth[1] + rng.gauss(0.0, std_xy),
        truth[2] + rng.gauss(0.0, std_z),
    )
    return noisy, 0, delayed


def run_propagation(
    profile: PropagationProfile,
    *,
    n_gt_callbacks: int = 36,
    seed: int = 101,
) -> PropagationCounts:
    timing_mod = _load_timing()
    range_mod = _load_range()
    fusion_mod = _load_fusion_module()
    tracking_mod, geom_mod = _load_tracking_module()

    radar_rng = random.Random(profile.radar_timing.seed if profile.radar_timing.seed >= 0 else seed)
    camera_rng = random.Random(profile.camera_timing.seed if profile.camera_timing.seed >= 0 else seed + 1)

    fusion = fusion_mod.FusionNode()
    fusion._pub.publish = MagicMock()

    truths: list[tuple[float, float, float]] = []
    fused_near_errors: list[float] = []
    fused_far_errors: list[float] = []

    counts = PropagationCounts()

    for idx in range(1, n_gt_callbacks + 1):
        counts.gt_callbacks += 1
        if idx <= n_gt_callbacks // 2:
            truth = (3.0 + 0.05 * idx, 1.0, 2.0)
        else:
            far = 0.85 * profile.radar_range.max_range_m
            truth = (far, 0.5, 1.5)
        truths.append(truth)

        radar_meas, dec_r, del_r = _sensor_publish(
            callback_index=idx,
            truth=truth,
            timing=profile.radar_timing,
            rng_cfg=profile.radar_range,
            timing_mod=timing_mod,
            range_mod=range_mod,
            rng=radar_rng,
        )
        counts.decimated_callbacks += dec_r
        counts.delayed_callbacks += del_r
        if radar_meas is not None:
            counts.radar_published += 1
            fusion._on_radar(_make_point(geom_mod, *radar_meas))

        camera_meas, dec_c, del_c = _sensor_publish(
            callback_index=idx,
            truth=truth,
            timing=profile.camera_timing,
            rng_cfg=profile.camera_range,
            timing_mod=timing_mod,
            range_mod=range_mod,
            rng=camera_rng,
        )
        counts.decimated_callbacks += dec_c
        counts.delayed_callbacks += del_c
        if camera_meas is not None:
            counts.camera_published += 1
            fusion._on_camera(_make_point(geom_mod, *camera_meas))

    counts.fused_published = fusion._pub.publish.call_count
    fused_points = [call.args[0] for call in fusion._pub.publish.call_args_list]

    tracking = tracking_mod.TrackingNode()
    tracking._confirmation_hits = 2
    tracking._candidate_match_gate_m = 20.0
    tracking._association_gate_m = 25.0
    tracking._candidate_max_missed_frames = 5
    tracking._pub.publish = MagicMock()
    state_pub = MagicMock()
    tracking._pub_state.publish = state_pub

    now_ns = 0
    n_cycles = max(40, len(fused_points) * 2)
    for cycle in range(n_cycles):
        if cycle < len(fused_points):
            tracking._on_detection(fused_points[cycle])
        now_ns += int(0.1 * 1e9)
        tracking.get_clock().now.return_value.nanoseconds = now_ns
        tracking._on_cycle_timer()

    counts.tracks_state_published = state_pub.call_count

    for i, det in enumerate(fused_points):
        truth_idx = min(i, len(truths) - 1)
        truth = truths[truth_idx]
        err = math.sqrt((det.x - truth[0]) ** 2 + (det.y - truth[1]) ** 2 + (det.z - truth[2]) ** 2)
        dist = math.sqrt(truth[0] ** 2 + truth[1] ** 2 + truth[2] ** 2)
        if dist < 0.5 * profile.radar_range.max_range_m:
            fused_near_errors.append(err)
        else:
            fused_far_errors.append(err)

    if fused_near_errors:
        counts.mean_fused_error_near_m = sum(fused_near_errors) / len(fused_near_errors)
    if fused_far_errors:
        counts.mean_fused_error_far_m = sum(fused_far_errors) / len(fused_far_errors)
    return counts
