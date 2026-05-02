import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


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
        # Phase 3 (L1): mimic camera pipeline latency without faking optics.
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
        self._delay_mean_s = max(0.0, float(self.get_parameter('camera.delay_mean_s').value))
        self._delay_jitter_s = max(0.0, float(self.get_parameter('camera.delay_jitter_s').value))

        self._scenario = str(self.get_parameter('scenario').value).strip().lower()
        if self._scenario == 'noisy':
            self._std_xy = 0.6
            self._std_z = 0.35
        else:
            self._std_xy = 0.2
            self._std_z = 0.1

        self._pub = self.create_publisher(Point, '/camera/detections', 10)
        self.create_subscription(Point, '/drone/position', self._on_position, 10)

    def _on_position(self, msg: Point) -> None:
        tx = msg.x - self._mx
        ty = msg.y - self._my
        tz = msg.z - self._mz
        # Rotate into camera frame: X_cam forward at yaw (world +X rotated by yaw to align boresight).
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
        if self._p_detect < 1.0 and random.random() > self._p_detect:
            self.get_logger().info('Camera missed detection (PD draw)')
            return

        out = Point()
        out.x = msg.x + random.gauss(0.0, self._std_xy)
        out.y = msg.y + random.gauss(0.0, self._std_xy)
        out.z = msg.z + random.gauss(0.0, self._std_z)
        self._publish_with_delay(out)
        self.get_logger().info('Camera detected target')

    def _publish_with_delay(self, out: Point) -> None:
        """Synchronous publish, or one-shot timer when delay > 0 (mirrors radar_sim_node)."""
        if self._delay_mean_s <= 0.0 and self._delay_jitter_s <= 0.0:
            self._pub.publish(out)
            return
        jitter = random.uniform(-self._delay_jitter_s, self._delay_jitter_s)
        delay_s = max(0.0, self._delay_mean_s + jitter)
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
