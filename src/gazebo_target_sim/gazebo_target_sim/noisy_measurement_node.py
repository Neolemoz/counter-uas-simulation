"""Add configurable measurement realism overlays to a Point topic."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import random

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node


def _sample_ghost_radius_m(
    rng: random.Random,
    *,
    placement_mode: str,
    broad_min_m: float,
    broad_max_m: float,
    near_threshold_min_m: float,
    near_threshold_max_m: float,
) -> float:
    if placement_mode == "near_threshold":
        return float(rng.uniform(near_threshold_min_m, near_threshold_max_m))
    return float(rng.uniform(broad_min_m, broad_max_m))


@dataclass(frozen=True)
class _GhostOffset:
    radius_m: float
    theta_rad: float
    z_offset_m: float


def _sample_ghost_offset(
    rng: random.Random,
    *,
    placement_mode: str,
    broad_min_m: float,
    broad_max_m: float,
    near_threshold_min_m: float,
    near_threshold_max_m: float,
    z_std_m: float,
) -> _GhostOffset:
    radius = _sample_ghost_radius_m(
        rng,
        placement_mode=placement_mode,
        broad_min_m=broad_min_m,
        broad_max_m=broad_max_m,
        near_threshold_min_m=near_threshold_min_m,
        near_threshold_max_m=near_threshold_max_m,
    )
    theta = float(rng.uniform(-math.pi, math.pi))
    z_offset = float(rng.gauss(0.0, z_std_m)) if z_std_m > 0.0 else 0.0
    return _GhostOffset(radius_m=radius, theta_rad=theta, z_offset_m=z_offset)


def _staggered_fragmentation_should_start(
    *,
    tick_index: int,
    cycle_ticks: int,
    phase_ticks: int,
) -> bool:
    if cycle_ticks <= 0 or tick_index < phase_ticks:
        return False
    return (tick_index - phase_ticks) % cycle_ticks == 0


@dataclass(frozen=True)
class _QueuedMeasurement:
    point: Point
    release_time_s: float
    source_age_s: float
    is_stale: bool
    applied_delay_s: float


class NoisyMeasurementNode(Node):
    """
    Subscribes to a ground-truth Point topic (default `/drone/position`) and publishes a
    noisy measurement Point (default `/drone/position_noisy`).

    This is a *measurement layer* (wiring realism). It does not change guidance math.
    """

    def __init__(self) -> None:
        super().__init__("noisy_measurement_node")
        self.declare_parameter("input_topic", "/drone/position")
        self.declare_parameter("output_topic", "/drone/position_noisy")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("noise_std_m", 0.0)
        self.declare_parameter("dropout_prob", 0.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("delay_mean_s", 0.0)
        self.declare_parameter("delay_jitter_s", 0.0)
        self.declare_parameter("publish_period_jitter_s", 0.0)
        self.declare_parameter("burst_dropout_prob", 0.0)
        self.declare_parameter("burst_dropout_min_ticks", 2)
        self.declare_parameter("burst_dropout_max_ticks", 5)
        self.declare_parameter("stale_detection_enabled", False)
        self.declare_parameter("stale_max_consecutive", 0)
        self.declare_parameter("ghost_detection_prob", 0.0)
        self.declare_parameter("ghost_placement_mode", "broad")
        self.declare_parameter("ghost_offset_xy_min_m", 3.0)
        self.declare_parameter("ghost_offset_xy_max_m", 12.0)
        self.declare_parameter("ghost_near_threshold_xy_min_m", 18.0)
        self.declare_parameter("ghost_near_threshold_xy_max_m", 24.0)
        self.declare_parameter("ghost_offset_z_std_m", 0.5)
        self.declare_parameter("ghost_persistence_ticks", 1)
        self.declare_parameter("fragmentation_prob", 0.0)
        self.declare_parameter("fragmentation_min_ticks", 2)
        self.declare_parameter("fragmentation_max_ticks", 5)
        self.declare_parameter("fragmentation_staggered_enabled", False)
        self.declare_parameter("fragmentation_stagger_cycle_ticks", 6)
        self.declare_parameter("fragmentation_stagger_phase_ticks", 1)
        self.declare_parameter("fragmentation_stagger_gap_ticks", 2)

        self._in_topic = str(self.get_parameter("input_topic").value).strip() or "/drone/position"
        self._out_topic = str(self.get_parameter("output_topic").value).strip() or "/drone/position_noisy"
        self._rate_hz = max(float(self.get_parameter("rate_hz").value), 0.1)
        self._noise_std = max(float(self.get_parameter("noise_std_m").value), 0.0)
        self._dropout = float(self.get_parameter("dropout_prob").value)
        self._dropout = min(max(self._dropout, 0.0), 1.0)
        self._seed = int(self.get_parameter("seed").value)
        self._delay_mean_s = max(float(self.get_parameter("delay_mean_s").value), 0.0)
        self._delay_jitter_s = max(float(self.get_parameter("delay_jitter_s").value), 0.0)
        self._period_jitter_s = max(float(self.get_parameter("publish_period_jitter_s").value), 0.0)
        self._burst_dropout_prob = min(
            max(float(self.get_parameter("burst_dropout_prob").value), 0.0),
            1.0,
        )
        self._burst_dropout_min_ticks = max(int(self.get_parameter("burst_dropout_min_ticks").value), 1)
        self._burst_dropout_max_ticks = max(
            int(self.get_parameter("burst_dropout_max_ticks").value),
            self._burst_dropout_min_ticks,
        )
        self._stale_enabled = bool(self.get_parameter("stale_detection_enabled").value)
        self._stale_max_consecutive = max(int(self.get_parameter("stale_max_consecutive").value), 0)
        self._ghost_detection_prob = min(
            max(float(self.get_parameter("ghost_detection_prob").value), 0.0),
            1.0,
        )
        ghost_mode = str(self.get_parameter("ghost_placement_mode").value).strip().lower()
        self._ghost_placement_mode = ghost_mode if ghost_mode in ("broad", "near_threshold") else "broad"
        self._ghost_offset_xy_min_m = max(float(self.get_parameter("ghost_offset_xy_min_m").value), 0.0)
        self._ghost_offset_xy_max_m = max(
            float(self.get_parameter("ghost_offset_xy_max_m").value),
            self._ghost_offset_xy_min_m,
        )
        self._ghost_near_threshold_xy_min_m = max(
            float(self.get_parameter("ghost_near_threshold_xy_min_m").value),
            0.0,
        )
        self._ghost_near_threshold_xy_max_m = max(
            float(self.get_parameter("ghost_near_threshold_xy_max_m").value),
            self._ghost_near_threshold_xy_min_m,
        )
        self._ghost_offset_z_std_m = max(float(self.get_parameter("ghost_offset_z_std_m").value), 0.0)
        self._ghost_persistence_ticks = max(int(self.get_parameter("ghost_persistence_ticks").value), 1)
        self._fragmentation_prob = min(
            max(float(self.get_parameter("fragmentation_prob").value), 0.0),
            1.0,
        )
        self._fragmentation_min_ticks = max(int(self.get_parameter("fragmentation_min_ticks").value), 1)
        self._fragmentation_max_ticks = max(
            int(self.get_parameter("fragmentation_max_ticks").value),
            self._fragmentation_min_ticks,
        )
        self._fragmentation_staggered_enabled = bool(
            self.get_parameter("fragmentation_staggered_enabled").value,
        )
        self._fragmentation_stagger_cycle_ticks = max(
            int(self.get_parameter("fragmentation_stagger_cycle_ticks").value),
            1,
        )
        self._fragmentation_stagger_phase_ticks = max(
            int(self.get_parameter("fragmentation_stagger_phase_ticks").value),
            0,
        )
        self._fragmentation_stagger_gap_ticks = max(
            int(self.get_parameter("fragmentation_stagger_gap_ticks").value),
            1,
        )

        self._rng = random.Random(self._seed)
        self._last_gt: Point | None = None
        self._last_fresh_sample: Point | None = None
        self._last_fresh_sample_time_s: float = 0.0
        self._pending: deque[_QueuedMeasurement] = deque()
        self._burst_ticks_remaining = 0
        self._fragmentation_ticks_remaining = 0
        self._consecutive_stale_emits = 0
        self._tick_timer = None
        self._event_delayed_count = 0
        self._event_stale_count = 0
        self._event_burst_count = 0
        self._event_ghost_count = 0
        self._event_fragmentation_count = 0
        self._fragmentation_active_mode: str | None = None
        self._active_ghost_offset: _GhostOffset | None = None
        self._active_ghost_ticks_remaining = 0
        self._tick_index = 0

        self._pub = self.create_publisher(Point, self._out_topic, 10)
        self.create_subscription(Point, self._in_topic, self._on_gt, 10)
        self._schedule_next_tick()

        self.get_logger().info(
            f"{self._in_topic} -> {self._out_topic} (Point) | rate={self._rate_hz:g} Hz "
            f"| noise_std_m={self._noise_std:g} | dropout_prob={self._dropout:g} | seed={self._seed} "
            f"| delay_mean_s={self._delay_mean_s:g} | delay_jitter_s={self._delay_jitter_s:g} "
            f"| publish_period_jitter_s={self._period_jitter_s:g} "
            f"| burst_dropout_prob={self._burst_dropout_prob:g} "
            f"| stale_detection_enabled={self._stale_enabled} "
            f"| ghost_detection_prob={self._ghost_detection_prob:g} "
            f"| ghost_placement_mode={self._ghost_placement_mode} "
            f"| ghost_persistence_ticks={self._ghost_persistence_ticks} "
            f"| fragmentation_prob={self._fragmentation_prob:g} "
            f"| fragmentation_staggered_enabled={self._fragmentation_staggered_enabled}",
        )

    def _on_gt(self, msg: Point) -> None:
        self._last_gt = msg

    def _gauss(self) -> float:
        # random.Random.gauss is deterministic for a given seed.
        return float(self._rng.gauss(0.0, self._noise_std)) if self._noise_std > 0.0 else 0.0

    def _schedule_next_tick(self) -> None:
        delay_s = 1.0 / self._rate_hz
        if self._period_jitter_s > 0.0:
            delay_s = max(1e-3, delay_s + self._rng.uniform(-self._period_jitter_s, self._period_jitter_s))
        timer_holder: list = [None]

        def _fire() -> None:
            t = timer_holder[0]
            if t is not None:
                t.cancel()
                self.destroy_timer(t)
            self._tick_timer = None
            self._tick()
            self._schedule_next_tick()

        timer_holder[0] = self.create_timer(delay_s, _fire)
        self._tick_timer = timer_holder[0]

    def _tick(self) -> None:
        self._tick_index += 1
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self._flush_pending(now_s)
        if self._last_gt is None:
            return

        if self._should_drop_current_tick():
            self._maybe_enqueue_stale(now_s)
            self._flush_pending(now_s)
            return

        p = Point()
        p.x = float(self._last_gt.x + self._gauss())
        p.y = float(self._last_gt.y + self._gauss())
        p.z = float(self._last_gt.z + self._gauss())
        if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
            return
        self._last_fresh_sample = self._copy_point(p)
        self._last_fresh_sample_time_s = now_s
        self._consecutive_stale_emits = 0
        self._enqueue_measurement(p, now_s, source_age_s=0.0, is_stale=False)
        self._maybe_enqueue_ghost(now_s)
        self._flush_pending(now_s)

    def _should_drop_current_tick(self) -> bool:
        if self._fragmentation_ticks_remaining > 0:
            self._fragmentation_ticks_remaining -= 1
            if self._fragmentation_ticks_remaining == 0:
                active_mode = self._fragmentation_active_mode or 'unknown'
                self.get_logger().info(
                    '[REALISM_EVENT] fragmented_gap_end '
                    f'mode={active_mode} '
                    f'tick_index={self._tick_index} '
                    f'seed={self._seed}',
                )
                self._fragmentation_active_mode = None
            return True
        if self._fragmentation_staggered_enabled and _staggered_fragmentation_should_start(
            tick_index=self._tick_index,
            cycle_ticks=self._fragmentation_stagger_cycle_ticks,
            phase_ticks=self._fragmentation_stagger_phase_ticks,
        ):
            self._fragmentation_ticks_remaining = self._fragmentation_stagger_gap_ticks
            self._event_fragmentation_count += 1
            self._fragmentation_active_mode = 'staggered'
            self.get_logger().info(
                '[REALISM_EVENT] fragmented_gap_start '
                f'fragmentation_index={self._event_fragmentation_count} '
                f'gap_ticks={self._fragmentation_ticks_remaining} '
                'mode=staggered '
                f'tick_index={self._tick_index} '
                f'cycle_ticks={self._fragmentation_stagger_cycle_ticks} '
                f'phase_ticks={self._fragmentation_stagger_phase_ticks} '
                f'seed={self._seed}',
            )
            self._fragmentation_ticks_remaining -= 1
            if self._fragmentation_ticks_remaining == 0:
                self.get_logger().info(
                    '[REALISM_EVENT] fragmented_gap_end '
                    'mode=staggered '
                    f'tick_index={self._tick_index} '
                    f'seed={self._seed}',
                )
                self._fragmentation_active_mode = None
            return True
        if self._fragmentation_prob > 0.0 and self._rng.random() < self._fragmentation_prob:
            self._fragmentation_ticks_remaining = self._rng.randint(
                self._fragmentation_min_ticks,
                self._fragmentation_max_ticks,
            )
            self._event_fragmentation_count += 1
            self._fragmentation_active_mode = 'random'
            self.get_logger().info(
                '[REALISM_EVENT] fragmented_gap_start '
                f'fragmentation_index={self._event_fragmentation_count} '
                f'gap_ticks={self._fragmentation_ticks_remaining} '
                'mode=random '
                f'tick_index={self._tick_index} '
                f'seed={self._seed}',
            )
            self._fragmentation_ticks_remaining -= 1
            if self._fragmentation_ticks_remaining == 0:
                self.get_logger().info(
                    '[REALISM_EVENT] fragmented_gap_end '
                    'mode=random '
                    f'tick_index={self._tick_index} '
                    f'seed={self._seed}',
                )
                self._fragmentation_active_mode = None
            return True
        if self._burst_ticks_remaining > 0:
            self._burst_ticks_remaining -= 1
            return True
        if self._burst_dropout_prob > 0.0 and self._rng.random() < self._burst_dropout_prob:
            self._burst_ticks_remaining = self._rng.randint(
                self._burst_dropout_min_ticks,
                self._burst_dropout_max_ticks,
            ) - 1
            self._event_burst_count += 1
            self.get_logger().info(
                '[REALISM_EVENT] dropout_burst_start '
                f'burst_index={self._event_burst_count} '
                f'burst_ticks={self._burst_ticks_remaining + 1}',
            )
            return True
        return self._dropout > 0.0 and self._rng.random() < self._dropout

    def _maybe_enqueue_stale(self, now_s: float) -> None:
        if self._fragmentation_ticks_remaining > 0:
            return
        if not self._stale_enabled or self._last_fresh_sample is None:
            return
        if self._consecutive_stale_emits >= self._stale_max_consecutive:
            return
        stale = self._copy_point(self._last_fresh_sample)
        age_s = max(0.0, now_s - self._last_fresh_sample_time_s)
        self._consecutive_stale_emits += 1
        self._event_stale_count += 1
        self._enqueue_measurement(stale, now_s, source_age_s=age_s, is_stale=True)

    def _maybe_enqueue_ghost(self, now_s: float) -> None:
        if self._ghost_detection_prob <= 0.0 or self._last_gt is None:
            return
        if self._active_ghost_ticks_remaining <= 0:
            if self._rng.random() >= self._ghost_detection_prob:
                return
            self._active_ghost_offset = _sample_ghost_offset(
                self._rng,
                placement_mode=self._ghost_placement_mode,
                broad_min_m=self._ghost_offset_xy_min_m,
                broad_max_m=self._ghost_offset_xy_max_m,
                near_threshold_min_m=self._ghost_near_threshold_xy_min_m,
                near_threshold_max_m=self._ghost_near_threshold_xy_max_m,
                z_std_m=self._ghost_offset_z_std_m,
            )
            self._active_ghost_ticks_remaining = self._ghost_persistence_ticks
        if self._active_ghost_offset is None:
            return
        offset = self._active_ghost_offset
        ghost = Point()
        ghost.x = float(self._last_gt.x + offset.radius_m * math.cos(offset.theta_rad) + self._gauss())
        ghost.y = float(self._last_gt.y + offset.radius_m * math.sin(offset.theta_rad) + self._gauss())
        ghost.z = float(
            self._last_gt.z
            + offset.z_offset_m
            + self._gauss()
        )
        if not (math.isfinite(ghost.x) and math.isfinite(ghost.y) and math.isfinite(ghost.z)):
            return
        self._event_ghost_count += 1
        self._enqueue_measurement(ghost, now_s, source_age_s=0.0, is_stale=False)
        persistence_remaining = self._active_ghost_ticks_remaining
        self._active_ghost_ticks_remaining -= 1
        if self._active_ghost_ticks_remaining <= 0:
            self._active_ghost_offset = None
        self.get_logger().info(
            '[REALISM_EVENT] ghost_detection '
            f'count={self._event_ghost_count} '
            f'offset_xy_m={offset.radius_m:.3f} '
            f'placement={self._ghost_placement_mode} '
            f'persistence_ticks={self._ghost_persistence_ticks} '
            f'persistence_remaining={persistence_remaining} '
            f'tick_index={self._tick_index} '
            f'seed={self._seed}',
        )

    def _enqueue_measurement(
        self,
        point: Point,
        now_s: float,
        *,
        source_age_s: float,
        is_stale: bool,
    ) -> None:
        release_time_s = now_s
        if self._delay_mean_s > 0.0 or self._delay_jitter_s > 0.0:
            jitter = self._rng.uniform(-self._delay_jitter_s, self._delay_jitter_s)
            release_time_s = max(now_s, now_s + self._delay_mean_s + jitter)
            self._event_delayed_count += 1
        self._pending.append(
            _QueuedMeasurement(
                point=self._copy_point(point),
                release_time_s=release_time_s,
                source_age_s=source_age_s,
                is_stale=is_stale,
                applied_delay_s=max(0.0, release_time_s - now_s),
            )
        )

    def _flush_pending(self, now_s: float) -> None:
        while self._pending and self._pending[0].release_time_s <= now_s:
            item = self._pending.popleft()
            self._pub.publish(item.point)
            if item.is_stale:
                self.get_logger().info(
                    '[REALISM_EVENT] stale_detection '
                    f'count={self._event_stale_count} '
                    f'source_age_s={item.source_age_s:.3f}',
                )
            elif item.applied_delay_s > 1e-6:
                self.get_logger().info(
                    '[REALISM_EVENT] delayed_detection '
                    f'count={self._event_delayed_count} '
                    f'delay_s={item.applied_delay_s:.3f}',
                )

    @staticmethod
    def _copy_point(src: Point) -> Point:
        out = Point()
        out.x = float(src.x)
        out.y = float(src.y)
        out.z = float(src.z)
        return out


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NoisyMeasurementNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
