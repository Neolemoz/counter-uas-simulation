"""Add configurable noise/dropout to a Point measurement topic (Phase 2)."""

from __future__ import annotations

import math
import random

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node


class NoisyMeasurementNode(Node):
    """
    Subscribes to a ground-truth Point topic (default `/drone/position`) and publishes a
    noisy measurement Point (default `/drone/position_noisy`).

    This is a *measurement layer* (wiring realism). It does not change guidance math.
    """

    def __init__(self) -> None:
        super().__init__("noisy_measurement_node")
        self.declare_parameter("input_topic", "/drone/position")
        self.declare_parameter("output_topic", "/drone/position_noisy")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("noise_std_m", 0.0)
        self.declare_parameter("dropout_prob", 0.0)
        self.declare_parameter("seed", 0)

        self._in_topic = str(self.get_parameter("input_topic").value).strip() or "/drone/position"
        self._out_topic = str(self.get_parameter("output_topic").value).strip() or "/drone/position_noisy"
        self._rate_hz = max(float(self.get_parameter("rate_hz").value), 0.1)
        self._noise_std = max(float(self.get_parameter("noise_std_m").value), 0.0)
        self._dropout = float(self.get_parameter("dropout_prob").value)
        self._dropout = min(max(self._dropout, 0.0), 1.0)
        self._seed = int(self.get_parameter("seed").value)

        self._rng = random.Random(self._seed)
        self._last_gt: Point | None = None

        self._pub = self.create_publisher(Point, self._out_topic, 10)
        self.create_subscription(Point, self._in_topic, self._on_gt, 10)
        self.create_timer(1.0 / self._rate_hz, self._tick)

        self.get_logger().info(
            f"{self._in_topic} -> {self._out_topic} (Point) | rate={self._rate_hz:g} Hz "
            f"| noise_std_m={self._noise_std:g} | dropout_prob={self._dropout:g} | seed={self._seed}",
        )

    def _on_gt(self, msg: Point) -> None:
        self._last_gt = msg

    def _gauss(self) -> float:
        # random.Random.gauss is deterministic for a given seed.
        return float(self._rng.gauss(0.0, self._noise_std)) if self._noise_std > 0.0 else 0.0

    def _tick(self) -> None:
        if self._last_gt is None:
            return

        if self._dropout > 0.0 and self._rng.random() < self._dropout:
            return

        p = Point()
        p.x = float(self._last_gt.x + self._gauss())
        p.y = float(self._last_gt.y + self._gauss())
        p.z = float(self._last_gt.z + self._gauss())
        if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
            return
        self._pub.publish(p)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NoisyMeasurementNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

