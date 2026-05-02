import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String


# Protected asset at origin (meters)
_PROTECTED = (0.0, 0.0, 0.0)


class ThreatAssessmentNode(Node):
    """Maps track distance to a threat label: HIGH / MEDIUM / LOW."""

    def __init__(self) -> None:
        super().__init__('threat_assessment_node')
        self.declare_parameter('tracks_topic', '/tracks')
        self.declare_parameter('high_distance_m', 5.0)
        self.declare_parameter('medium_distance_m', 10.0)

        topic = str(self.get_parameter('tracks_topic').value).strip() or '/tracks'
        self._r_high = max(float(self.get_parameter('high_distance_m').value), 0.1)
        self._r_med = max(float(self.get_parameter('medium_distance_m').value), self._r_high + 0.1)

        self._pub = self.create_publisher(String, '/threat_level', 10)
        self.create_subscription(Point, topic, self._on_track, 10)
        self.get_logger().info(
            f'threat_assessment: tracks_topic={topic} high_m={self._r_high} medium_m={self._r_med}',
        )

    def _on_track(self, msg: Point) -> None:
        px, py, pz = _PROTECTED
        dx = msg.x - px
        dy = msg.y - py
        dz = msg.z - pz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        out = String()
        if dist < self._r_high:
            out.data = 'HIGH'
        elif dist < self._r_med:
            out.data = 'MEDIUM'
        else:
            out.data = 'LOW'
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ThreatAssessmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
