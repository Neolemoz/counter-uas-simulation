import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from camera_sim.range_realism import (
    effective_detection_probability,
    effective_measurement_std,
)
from camera_sim.timing_realism import should_publish_on_callback, transport_delay_s


def _deg2rad(d: float) -> float:
    return d * math.pi / 180.0


class CameraSimNode(Node):
    """Sector camera: mount pose, yaw boresight, horizontal / vertical FOV, max range, and PD."""

    def __init__(self) -> None:
        super().__init__('camera_sim_node')
        self.declare_parameter('scenario', 'single')
        self.declare_parameter('camera.mount_x', 0.0)
        self.declare_parameter('camera.mount_y', 0.0)
        self.declare_parameter('camera.mount_z', 0.0)
        self.declare_parameter('camera.yaw_deg', 0.0)
        self.declare_parameter('camera.horizontal_fov_deg', 90.0)
        self.declare_parameter('camera.vertical_fov_deg', 55.0)
        self.declare_parameter('camera.max_range_m', 500.0)
        self.declare_parameter('camera.detection_probability', 0.95)
        self.declare_parameter('camera.detection_probability_decay_with_range', 0.0)
        self.declare_parameter('camera.min_detection_probability', 0.0)
        self.declare_parameter('camera.measurement_std_scale_with_range', 0.0)
        self.declare_parameter('camera.publish_every_n', 1)
        self.declare_parameter('camera.seed', -1)
        self.declare_parameter('camera.delay_mean_s', 0.0)
        self.declare_parameter('camera.delay_jitter_s', 0.0)

        self._mx = float(self.get_parameter('camera.mount_x').value)
        self._my = float(self.get_parameter('camera.mount_y').value)
        self._mz = float(self.get_parameter('camera.mount_z').value)
        yaw = _deg2rad(float(self.get_parameter('camera.yaw_deg').value))
        self._cos_y = math.cos(yaw)
        self._sin_y = math.sin(yaw)
        self._hz = max(_deg2rad(float(self.get_parameter('camera.horizontal_fov_deg').value)), 1e-6)
        self._vz = max(_deg2rad(float(self.get_parameter('camera.vertical_fov_deg').value)), 1e-6)
        self._hz_half = self._hz / 2.0
        self._vz_half = self._vz / 2.0
        self._r_max = max(float(self.get_parameter('camera.max_range_m').value), 0.0)
        self._p_detect = min(1.0, max(0.0, float(self.get_parameter('camera.detection_probability').value)))
        self._pd_decay = max(
            0.0, float(self.get_parameter('camera.detection_probability_decay_with_range').value)
        )
        self._pd_min = min(1.0, max(0.0, float(self.get_parameter('camera.min_detection_probability').value)))
        self._std_scale_range = max(
            0.0, float(self.get_parameter('camera.measurement_std_scale_with_range').value)
        )
        self._publish_every_n = max(1, int(self.get_parameter('camera.publish_every_n').value))
        seed = int(self.get_parameter('camera.seed').value)
        self._rng: random.Random | None = random.Random(seed) if seed >= 0 else None
        self._delay_mean_s = max(0.0, float(self.get_parameter('camera.delay_mean_s').value))
        self._delay_jitter_s = max(0.0, float(self.get_parameter('camera.delay_jitter_s').value))

        self._scenario = str(self.get_parameter('scenario').value).strip().lower()
        if self._scenario == 'noisy':
            self._std_xy = 0.6
            self._std_z = 0.35
        else:
            self._std_xy = 0.2
            self._std_z = 0.1

        self._input_callback_count = 0
        self._pub = self.create_publisher(Point, '/camera/detections', 10)
        self.create_subscription(Point, '/drone/position', self._on_position, 10)

    def _rand(self) -> random.Random:
        return self._rng if self._rng is not None else random

    def _on_position(self, msg: Point) -> None:
        self._input_callback_count += 1
        if not should_publish_on_callback(self._input_callback_count, self._publish_every_n):
            return

        tx = msg.x - self._mx
        ty = msg.y - self._my
        tz = msg.z - self._mz
        cam_x = self._cos_y * tx + self._sin_y * ty
        cam_y = -self._sin_y * tx + self._cos_y * ty
        cam_z = tz
        rng = math.sqrt(cam_x * cam_x + cam_y * cam_y + cam_z * cam_z)
        if rng < 1e-6 or cam_x <= 0.0:
            self.get_logger().info('Camera missed target (behind or at mount)')
            return
        if self._r_max > 0.0 and rng > self._r_max:
            self.get_logger().info(f'Camera missed target (range {rng:.1f} m)')
            return
        az = math.atan2(cam_y, cam_x)
        el = math.atan2(cam_z, math.hypot(cam_x, cam_y))
        if abs(az) > self._hz_half or abs(el) > self._vz_half:
            self.get_logger().info('Camera missed target (outside FOV)')
            return
        p_eff = effective_detection_probability(
            base_p=self._p_detect,
            distance_m=rng,
            max_range_m=self._r_max,
            decay_with_range=self._pd_decay,
            min_detection_probability=self._pd_min,
        )
        if p_eff < 1.0 and self._rand().random() > p_eff:
            self.get_logger().info(
                f'Camera missed detection (PD draw) range={rng:.2f} m p_eff={p_eff:.3f}',
            )
            return

        std_xy = effective_measurement_std(
            self._std_xy, rng, self._r_max, self._std_scale_range
        )
        std_z = effective_measurement_std(
            self._std_z, rng, self._r_max, self._std_scale_range
        )
        out = Point()
        out.x = msg.x + self._rand().gauss(0.0, std_xy)
        out.y = msg.y + self._rand().gauss(0.0, std_xy)
        out.z = msg.z + self._rand().gauss(0.0, std_z)
        self._publish_with_delay(out)
        self.get_logger().info('Camera detected target')

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
    node = CameraSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
