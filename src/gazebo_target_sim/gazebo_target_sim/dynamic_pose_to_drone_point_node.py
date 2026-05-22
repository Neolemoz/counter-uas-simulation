"""Subscribe to bridged dynamic_pose (tf2_msgs/TFMessage) and republish target world position as Point."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class DynamicPoseToDronePointNode(Node):
    """
    ros_gz_bridge Pose_V -> TFMessage may leave child_frame_id empty; for this single-target
    world we take the transform with the largest translation norm (model world pose vs zeros).
    """

    def __init__(self) -> None:
        super().__init__('dynamic_pose_to_drone_point_node')
        self.declare_parameter('world_name', 'counter_uas_target')
        self.declare_parameter('model_child_frame', '')
        self.declare_parameter('output_topic', '/drone/position')
        self.declare_parameter('publish_period_s', 0.1)
        self.declare_parameter('prefer_highest_z', True)
        self.declare_parameter('min_translation_norm2', 0.01)
        world = str(self.get_parameter('world_name').value).strip()
        self._child = str(self.get_parameter('model_child_frame').value).strip()
        self._prefer_z = bool(self.get_parameter('prefer_highest_z').value)
        self._min_n2 = max(float(self.get_parameter('min_translation_norm2').value), 1e-9)
        out_topic = str(self.get_parameter('output_topic').value).strip()
        self._tf_topic = f'/world/{world}/dynamic_pose/info'
        self._period = max(float(self.get_parameter('publish_period_s').value), 1e-4)
        self._last_pub = self.get_clock().now()
        self._pub = self.create_publisher(Point, out_topic, 10)
        self.create_subscription(TFMessage, self._tf_topic, self._on_tf, 10)
        filt = repr(self._child) if self._child else 'OFF'
        zmode = 'max_z among ||t||>min' if self._prefer_z else 'max ||t||'
        self.get_logger().info(
            f'{self._tf_topic} -> {out_topic} (Point); child={filt}; fallback={zmode}',
        )

    @staticmethod
    def _norm2(ts: TransformStamped) -> float:
        t = ts.transform.translation
        return float(t.x * t.x + t.y * t.y + t.z * t.z)

    def _on_tf(self, msg: TFMessage) -> None:
        now = self.get_clock().now()
        if (now - self._last_pub).nanoseconds * 1e-9 < self._period:
            return

        chosen: TransformStamped | None = None
        if self._child:
            for ts in msg.transforms:
                if ts.child_frame_id == self._child:
                    chosen = ts
                    break
        if chosen is None:
            cands = [ts for ts in msg.transforms if self._norm2(ts) >= self._min_n2]
            if not cands:
                return
            if self._prefer_z:
                chosen = max(cands, key=lambda ts: ts.transform.translation.z)
            else:
                chosen = max(cands, key=self._norm2)
        if chosen is None:
            return
        t = chosen.transform.translation
        p = Point()
        p.x = float(t.x)
        p.y = float(t.y)
        p.z = float(t.z)
        self._pub.publish(p)
        self._last_pub = now


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DynamicPoseToDronePointNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
