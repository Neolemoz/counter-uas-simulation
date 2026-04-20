"""Detect Gazebo reset: bridged ``/clock`` sim time jumps backward (e.g. GUI Reset)."""

from __future__ import annotations

from collections.abc import Callable

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def subscribe_sim_time_reset(
    node: Node,
    on_reset: Callable[[], None],
    *,
    min_backward_ns: int = 50_000_000,
) -> None:
    """
    Subscribe to ``/clock`` (จาก ros_gz_bridge + gz sim).

    เมื่อเวลาจำลองย้อนหลังมากกว่า ``min_backward_ns`` ถือว่า user กด reset — เรียก ``on_reset``.
    """
    last_ns: list[int | None] = [None]

    def _cb(msg: Clock) -> None:
        t_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        if last_ns[0] is not None and t_ns < last_ns[0] - min_backward_ns:
            on_reset()
        last_ns[0] = t_ns

    qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    node.create_subscription(Clock, '/clock', _cb, qos)
