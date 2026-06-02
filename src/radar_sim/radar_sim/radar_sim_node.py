import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from radar_sim.range_realism import (
    effective_detection_probability,
    effective_measurement_std,
)
from radar_sim.timing_realism import should_publish_on_callback, transport_delay_s


def _deg2rad(d: float) -> float:
    return d * math.pi / 180.0


class RadarSimNode(Node):
    """
    Subscribes to true drone position and publishes noisy radar-style detections.

    Output rate matches the input rate (one detection per incoming message) unless
    ``radar.publish_every_n`` > 1. Optional angular gating and Bernoulli missed detections.
    """

    def __init__(self) -> None:
        super().__init__('radar_sim_node')
        self.declare_parameter('scenario', 'single')
        self.declare_parameter('radar.range', 12.0)
        self.declare_parameter('radar.position_x', 0.0)
        self.declare_parameter('radar.position_y', 0.0)
        self.declare_parameter('radar.position_z', 0.0)
        self.declare_parameter('radar.azimuth_half_deg', 90.0)
        self.declare_parameter('radar.elevation_half_deg', 60.0)
        self.declare_parameter('radar.detection_probability', 1.0)
        self.declare_parameter('radar.detection_probability_decay_with_range', 0.0)
        self.declare_parameter('radar.min_detection_probability', 0.0)
        self.declare_parameter('radar.measurement_std_scale_with_range', 0.0)
        self.declare_parameter('radar.publish_every_n', 1)
        self.declare_parameter('radar.seed', -1)
        self.declare_parameter('radar.delay_mean_s', 0.0)
        self.declare_parameter('radar.delay_jitter_s', 0.0)

        self._range_m = float(self.get_parameter('radar.range').value)
        self._px = float(self.get_parameter('radar.position_x').value)
        self._py = float(self.get_parameter('radar.position_y').value)
        self._pz = float(self.get_parameter('radar.position_z').value)
        self._az_half = max(_deg2rad(float(self.get_parameter('radar.azimuth_half_deg').value)), 1e-6)
        self._el_half = max(_deg2rad(float(self.get_parameter('radar.elevation_half_deg').value)), 1e-6)
        self._p_detect = min(1.0, max(0.0, float(self.get_parameter('radar.detection_probability').value)))
        self._pd_decay = max(0.0, float(self.get_parameter('radar.detection_probability_decay_with_range').value))
        self._pd_min = min(1.0, max(0.0, float(self.get_parameter('radar.min_detection_probability').value)))
        self._std_scale_range = max(
            0.0, float(self.get_parameter('radar.measurement_std_scale_with_range').value)
        )
        self._publish_every_n = max(1, int(self.get_parameter('radar.publish_every_n').value))
        seed = int(self.get_parameter('radar.seed').value)
        self._rng: random.Random | None = random.Random(seed) if seed >= 0 else None
        self._delay_mean_s = max(0.0, float(self.get_parameter('radar.delay_mean_s').value))
        self._delay_jitter_s = max(0.0, float(self.get_parameter('radar.delay_jitter_s').value))

        self._scenario = str(self.get_parameter('scenario').value).strip().lower()
        if self._scenario == 'noisy':
            self._std_xy = 1.0
            self._std_z = 1.0
        else:
            self._std_xy = 0.5
            self._std_z = 0.2

        self._input_callback_count = 0
        self._pub = self.create_publisher(Point, '/radar/detections', 10)
        self.create_subscription(Point, '/drone/position', self._on_position, 10)

    def _rand(self) -> random.Random:
        return self._rng if self._rng is not None else random

    def _in_beam(self, msg: Point) -> bool:
        dx = msg.x - self._px
        dy = msg.y - self._py
        dz = msg.z - self._pz
        rng = math.sqrt(dx * dx + dy * dy + dz * dz)
        if rng < 1e-9:
            return True
        az = math.atan2(dy, dx)
        el = math.asin(max(-1.0, min(1.0, dz / rng)))
        return abs(az) <= self._az_half and abs(el) <= self._el_half

    def _on_position(self, msg: Point) -> None:
        self._input_callback_count += 1
        if not should_publish_on_callback(self._input_callback_count, self._publish_every_n):
            return

        dx = msg.x - self._px
        dy = msg.y - self._py
        dz = msg.z - self._pz
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance > self._range_m:
            self.get_logger().info(f'Target out of range: distance={distance:.2f} m')
            return
        if not self._in_beam(msg):
            self.get_logger().info(
                f'Target outside radar beam: distance={distance:.2f} m',
            )
            return
        p_eff = effective_detection_probability(
            base_p=self._p_detect,
            distance_m=distance,
            max_range_m=self._range_m,
            decay_with_range=self._pd_decay,
            min_detection_probability=self._pd_min,
        )
        if p_eff < 1.0 and self._rand().random() > p_eff:
            self.get_logger().info(
                f'Radar missed detection (PD draw) distance={distance:.2f} m p_eff={p_eff:.3f}',
            )
            return

        self.get_logger().info(f'Detected target at distance={distance:.2f} m')

        std_xy = effective_measurement_std(
            self._std_xy, distance, self._range_m, self._std_scale_range
        )
        std_z = effective_measurement_std(
            self._std_z, distance, self._range_m, self._std_scale_range
        )
        out = Point()
        out.x = msg.x + self._rand().gauss(0.0, std_xy)
        out.y = msg.y + self._rand().gauss(0.0, std_xy)
        out.z = msg.z + self._rand().gauss(0.0, std_z)
        self._publish_with_delay(out)

    def _publish_with_delay(self, out: Point) -> None:
        delay_s = transport_delay_s(self._delay_mean_s, self._delay_jitter_s, self._rng)
        if delay_s <= 1e-4:
            self._pub.publish(out)
            return

        captured = Point()
        captured.x, captured.y, captured.z = out.x, out.y, out.z
        timer_holder: list = [None]

        def _fire() -> None:
            self._pub.publish(captured)
            t = timer_holder[0]
            if t is not None:
                t.cancel()
                self.destroy_timer(t)

        timer_holder[0] = self.create_timer(delay_s, _fire)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadarSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
