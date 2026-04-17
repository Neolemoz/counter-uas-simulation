import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# After this many seconds, switch to the inner-orbit drone (passes through the zone).
SWITCH_S = 5.0
# Scale time in sin/cos so orbits sweep faster (quicker HIGH / MEDIUM transitions).
ORBIT_SPEED = 3.0
# dropout scenario: probability to skip publishing /drone/position this tick
DROPOUT_SKIP_P = 0.35


class WorldSimNode(Node):
    """Publishes drone position(s) and RViz markers at 10 Hz."""

    def __init__(self):
        super().__init__('world_sim_node')
        self.declare_parameter('scenario', 'single')
        self._scenario = str(self.get_parameter('scenario').value).strip().lower()

        self._pub_point = self.create_publisher(Point, '/drone/position', 10)
        self._pub_marker = self.create_publisher(Marker, '/drone/marker', 10)
        self._timer = self.create_timer(0.1, self._on_timer)  # 10 Hz
        self._t0 = self.get_clock().now()

        self.get_logger().info(f'Scenario: {self._scenario}')

    def _on_timer(self) -> None:
        self.t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        tau = ORBIT_SPEED * self.t
        stamp = self.get_clock().now().to_msg()

        if self._scenario == 'dual':
            self._tick_dual(tau, stamp)
            return

        if self._scenario == 'dropout':
            if random.random() < DROPOUT_SKIP_P:
                self.get_logger().info('dropout: skipped publish')
                return

        # single, noisy, dropout (after skip check): same motion as original "single"
        self._tick_single(tau, stamp)

    def _tick_single(self, tau: float, stamp) -> None:
        x1 = 10.0 * math.cos(tau)
        y1 = 10.0 * math.sin(tau)
        z1 = 5.0
        x2 = 5.0 * math.cos(tau + 1.0) + 5.0
        y2 = 5.0 * math.sin(tau + 1.0)
        z2 = 5.0

        if self.t >= SWITCH_S:
            msg2 = Point()
            msg2.x = x2
            msg2.y = y2
            msg2.z = z2
            self._pub_point.publish(msg2)
            self.get_logger().info(
                f't={self.t:.2f} Drone 2 only: ({msg2.x:.3f}, {msg2.y:.3f}, {msg2.z:.3f})',
            )
            self._publish_sphere_marker(stamp, 1, x2, y2, z2, 0.0, 0.4, 1.0)
        else:
            p1 = Point()
            p1.x = x1
            p1.y = y1
            p1.z = z1
            self._pub_point.publish(p1)
            self.get_logger().info(
                f't={self.t:.2f} Drone 1 only: ({p1.x:.3f}, {p1.y:.3f}, {p1.z:.3f})',
            )
            self._publish_sphere_marker(stamp, 0, x1, y1, z1, 1.0, 0.0, 0.0)

    def _tick_dual(self, tau: float, stamp) -> None:
        # Two drones: outer ring + offset inner-like path (different phase/radius).
        xa = 10.0 * math.cos(tau)
        ya = 10.0 * math.sin(tau)
        za = 5.0
        xb = 7.0 * math.cos(tau + 2.2) + 3.0
        yb = 7.0 * math.sin(tau + 2.2)
        zb = 5.0

        pa = Point()
        pa.x, pa.y, pa.z = xa, ya, za
        pb = Point()
        pb.x, pb.y, pb.z = xb, yb, zb
        self._pub_point.publish(pa)
        self._pub_point.publish(pb)
        self.get_logger().info(
            f't={self.t:.2f} dual: A=({xa:.2f},{ya:.2f}) B=({xb:.2f},{yb:.2f})',
        )
        self._publish_sphere_marker(stamp, 0, xa, ya, za, 1.0, 0.0, 0.0)
        self._publish_sphere_marker(stamp, 1, xb, yb, zb, 0.0, 0.8, 0.2)

    def _publish_sphere_marker(
        self,
        stamp,
        marker_id: int,
        x: float,
        y: float,
        z: float,
        r: float,
        g: float,
        b: float,
    ) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'
        marker.ns = 'drone'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        self._pub_marker.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
