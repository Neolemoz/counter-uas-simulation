import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class FusionNode(Node):
    """Fuses the latest radar and camera Points into /fused_detections."""

    def __init__(self) -> None:
        super().__init__('fusion_node')
        self.declare_parameter('scenario', 'single')
        self.declare_parameter('fusion.distance_threshold', 6.0)
        self._distance_threshold = float(self.get_parameter('fusion.distance_threshold').value)

        self._pub = self.create_publisher(Point, '/fused_detections', 10)

        self._latest_radar: Point | None = None
        self._latest_camera: Point | None = None

        self.create_subscription(Point, '/radar/detections', self._on_radar, 10)
        self.create_subscription(Point, '/camera/detections', self._on_camera, 10)

    def _on_radar(self, msg: Point) -> None:
        self._latest_radar = self._copy_point(msg)
        self._publish_fused()

    def _on_camera(self, msg: Point) -> None:
        self._latest_camera = self._copy_point(msg)
        self._publish_fused()

    def _publish_fused(self) -> None:
        r = self._latest_radar
        c = self._latest_camera

        if r is not None and c is not None:
            d = math.sqrt((r.x - c.x) ** 2 + (r.y - c.y) ** 2 + (r.z - c.z) ** 2)
            # Debug: show how close the two sensors are (every time both exist)
            self.get_logger().info(f'Distance radar-camera: {d:.2f}')
            # Increased threshold to make fusion easier to trigger
            if d < self._distance_threshold:
                out = Point()
                out.x = (r.x + c.x) / 2.0
                out.y = (r.y + c.y) / 2.0
                out.z = (r.z + c.z) / 2.0
                self._pub.publish(out)
                self.get_logger().info('Fused radar + camera')
            else:
                # Too far apart: publish both separately
                self._pub.publish(r)
                self._pub.publish(c)
                self.get_logger().info('Radar only')
                self.get_logger().info('Camera only')
            return

        if r is not None:
            self._pub.publish(r)
            self.get_logger().info('Radar only')
            return

        if c is not None:
            self._pub.publish(c)
            self.get_logger().info('Camera only')

    @staticmethod
    def _copy_point(msg: Point) -> Point:
        p = Point()
        p.x, p.y, p.z = msg.x, msg.y, msg.z
        return p


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
