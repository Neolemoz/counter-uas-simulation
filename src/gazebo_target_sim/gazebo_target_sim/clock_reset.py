"""Detect Gazebo reset: bridged ``/clock`` sim time jumps backward (e.g. GUI Reset)."""

from __future__ import annotations

import time
from collections.abc import Callable

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def subscribe_sim_time_reset(
    node: Node,
    on_reset: Callable[[], None],
    *,
    min_backward_ns: int = 900_000_000,
) -> None:
    """
    Subscribe to ``/clock`` (จาก ros_gz_bridge + gz sim).

    เมื่อเวลาจำลองย้อนหลังมากกว่า ``min_backward_ns`` **เมื่อเทียบกับค่าสูงสุดที่เคยเห็น**
    ถือว่า user กด reset — เรียก ``on_reset``.

    เฉพาะการย้อนเมื่อเทียบข้อความก่อนหน้าไม่พอ: ข้อความ /clock แบบ best-effort อาจสลับลำดับ
    ย้อนหลัง ~100 ms ทำให้เกิด on_reset เป็นหมื่นครั้ง/นาที (ทุกโหนด + transport พัง)
    """
    last_ns: list[int | None] = [None]
    peak_ns: list[int] = [0]
    last_on_reset_wall_s: list[float] = [0.0]
    debounce_wall_s = 0.6

    def _cb(msg: Clock) -> None:
        t_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        if t_ns > peak_ns[0]:
            peak_ns[0] = t_ns
        rollback = peak_ns[0] - t_ns
        if rollback > min_backward_ns:
            # Near t≈0 ⇒ GUI reset to empty sim; else require a large rollback (seconds) so
            # out-of-order /clock frames (sub-second) do not wipe state every tick.
            # (Do not use "t < 3 s" — early engagement would look like a rewind from a stale frame.)
            near_zero = t_ns < int(300_000_000)
            big_jump = rollback >= max(min_backward_ns, int(5_000_000_000))
            if near_zero or big_jump:
                now_wall = time.monotonic()
                if now_wall - last_on_reset_wall_s[0] < debounce_wall_s:
                    last_ns[0] = t_ns
                    return
                last_on_reset_wall_s[0] = now_wall
                on_reset()
                peak_ns[0] = t_ns
        last_ns[0] = t_ns

    # Use a large backward window: BEST_EFFORT /clock can be reordered slightly; small thresholds
    # look like GUI "Reset" and spam Sim reset (clears guidance + transport load).
    qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    node.create_subscription(Clock, '/clock', _cb, qos)
