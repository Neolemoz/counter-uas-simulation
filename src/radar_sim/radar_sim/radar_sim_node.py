import random
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class RadarSimNode(Node):
    """
    Subscribes to true drone position and publishes noisy radar-style detections.

    Output rate matches the input rate (one detection per incoming message).
    """

    def __init__(self):
        super().__init__('radar_sim_node')
        self.declare_parameter('scenario', 'single')
        self.declare_parameter('radar.range', 12.0)
        self._range_m = float(self.get_parameter('radar.range').value)
        self._scenario = str(self.get_parameter('scenario').value).strip().lower()
        # Default noise vs noisy scenario
        if self._scenario == 'noisy':
            self._std_xy = 1.0
            self._std_z = 1.0
        else:
            self._std_xy = 0.5
            self._std_z = 0.2

        self._pub = self.create_publisher(Point, '/radar/detections', 10)
        self.create_subscription(Point, '/drone/position', self._on_position, 10)

    def _on_position(self, msg: Point) -> None:
        # --- Range filtering (radar at origin) ---
        distance = math.sqrt(msg.x * msg.x + msg.y * msg.y + msg.z * msg.z)
        if distance > self._range_m:
            self.get_logger().info(f'Target out of range: distance={distance:.2f} m')
            return
        self.get_logger().info(f'Detected target at distance={distance:.2f} m')

        out = Point()
        out.x = msg.x + random.gauss(0.0, self._std_xy)
        out.y = msg.y + random.gauss(0.0, self._std_xy)
        out.z = msg.z + random.gauss(0.0, self._std_z)
        self._pub.publish(out)


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
