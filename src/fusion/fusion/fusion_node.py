"""Single-output sensor fusion: emits exactly one ``geometry_msgs/Point`` per fusion event.

Previous behaviour double-published radar and camera as **separate** outputs when sensors
disagreed (distance > threshold).  Downstream consumers (`tracking_node`, `interception_logic_node`)
treat ``/fused_detections`` as a single coherent measurement stream, so dual publishes broke
both association and engagement state.  The disagreement strategy is now an explicit
parameter (``prefer_radar`` | ``prefer_camera`` | ``drop``); the default mirrors the prior
"radar is the more trusted range source" assumption.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node


class FusionNode(Node):
    """Fuses the latest radar and camera Points into ``/fused_detections``."""

    def __init__(self) -> None:
        super().__init__('fusion_node')
        self.declare_parameter('scenario', 'single')
        self.declare_parameter('fusion.distance_threshold', 6.0)
        self.declare_parameter('fusion.radar_weight', 1.0)
        self.declare_parameter('fusion.camera_weight', 1.0)
        # Disagreement strategy: how to emit when |r - c| > threshold.
        # ``prefer_radar`` (default): radar typically gives better range; trust it for engagement.
        # ``prefer_camera``: useful when camera has lower angular noise indoors.
        # ``drop``: skip publishing this cycle — downstream sees no measurement, association
        # gates / track miss counters handle it cleanly.
        self.declare_parameter('fusion.disagreement_strategy', 'prefer_radar')

        self._distance_threshold = float(self.get_parameter('fusion.distance_threshold').value)
        self._w_r = max(float(self.get_parameter('fusion.radar_weight').value), 1e-9)
        self._w_c = max(float(self.get_parameter('fusion.camera_weight').value), 1e-9)
        strat = str(self.get_parameter('fusion.disagreement_strategy').value).strip().lower()
        if strat not in ('prefer_radar', 'prefer_camera', 'drop'):
            self.get_logger().warning(
                f'Unknown fusion.disagreement_strategy={strat!r}; falling back to prefer_radar.',
            )
            strat = 'prefer_radar'
        self._disagree = strat

        self._pub = self.create_publisher(Point, '/fused_detections', 10)

        self._latest_radar: Point | None = None
        self._latest_camera: Point | None = None

        self.create_subscription(Point, '/radar/detections', self._on_radar, 10)
        self.create_subscription(Point, '/camera/detections', self._on_camera, 10)

        self.get_logger().info(
            f'Fusion: distance_threshold={self._distance_threshold:.2f}m  '
            f'weights(radar={self._w_r:.2f}, camera={self._w_c:.2f})  '
            f'disagreement_strategy={self._disagree}',
        )

    def _on_radar(self, msg: Point) -> None:
        self._latest_radar = self._copy_point(msg)
        self._publish_fused()

    def _on_camera(self, msg: Point) -> None:
        self._latest_camera = self._copy_point(msg)
        self._publish_fused()

    def _fuse_points(self, r: Point, c: Point) -> Point:
        w = self._w_r + self._w_c
        out = Point()
        out.x = (self._w_r * r.x + self._w_c * c.x) / w
        out.y = (self._w_r * r.y + self._w_c * c.y) / w
        out.z = (self._w_r * r.z + self._w_c * c.z) / w
        return out

    def _publish_fused(self) -> None:
        r = self._latest_radar
        c = self._latest_camera

        if r is not None and c is not None:
            d = math.sqrt((r.x - c.x) ** 2 + (r.y - c.y) ** 2 + (r.z - c.z) ** 2)
            if d < self._distance_threshold:
                out = self._fuse_points(r, c)
                self._pub.publish(out)
                self.get_logger().info(
                    f'Fused radar+camera (weighted mean)  delta={d:.2f}m',
                )
                return
            # Sensors disagree: emit at most one point so downstream sees one coherent measurement.
            if self._disagree == 'prefer_radar':
                self._pub.publish(r)
                self.get_logger().info(
                    f'Disagreement (delta={d:.2f}m): published radar (prefer_radar)',
                )
            elif self._disagree == 'prefer_camera':
                self._pub.publish(c)
                self.get_logger().info(
                    f'Disagreement (delta={d:.2f}m): published camera (prefer_camera)',
                )
            else:
                self.get_logger().info(
                    f'Disagreement (delta={d:.2f}m): dropped (drop strategy)',
                )
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
