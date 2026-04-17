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
        self._pub = self.create_publisher(String, '/threat_level', 10)
        self.create_subscription(Point, '/tracks', self._on_track, 10)

    def _on_track(self, msg: Point) -> None:
        px, py, pz = _PROTECTED
        dx = msg.x - px
        dy = msg.y - py
        dz = msg.z - pz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        out = String()
        if dist < 5.0:
            out.data = 'HIGH'
        elif dist < 10.0:
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
