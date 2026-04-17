import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class CameraSimNode(Node):
    """Publishes low-noise camera detections when the target is in front (x > 0)."""

    def __init__(self) -> None:
        super().__init__('camera_sim_node')
        self.declare_parameter('scenario', 'single')
        self._pub = self.create_publisher(Point, '/camera/detections', 10)
        self.create_subscription(Point, '/drone/position', self._on_position, 10)

    def _on_position(self, msg: Point) -> None:
        # Simple field-of-view: camera faces forward, only sees targets with x > 0.
        if msg.x <= 0.0:
            self.get_logger().info('Camera missed target')
            return

        out = Point()
        out.x = msg.x + random.gauss(0.0, 0.2)
        out.y = msg.y + random.gauss(0.0, 0.2)
        out.z = msg.z + random.gauss(0.0, 0.1)
        self._pub.publish(out)
        self.get_logger().info('Camera detected target')


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
