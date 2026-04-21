#!/usr/bin/env python3
"""
Smoke test: interception_logic_node + fake /drone/position and interceptor positions.
No Gazebo. Captures [Intercept Debug] lines to stdout.
"""
from __future__ import annotations

import os
import select
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node


def main() -> int:
    ws = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    if not ws:
        print('Source install/setup.bash first (COLCON_PREFIX_PATH unset).', file=sys.stderr)
        return 1

    rclpy.init()
    fake = Node('smoke_intercept_debug_pub')
    pub_t = fake.create_publisher(Point, '/drone/position', 10)
    pubs = [fake.create_publisher(Point, f'/interceptor_{i}/position', 10) for i in range(3)]

    t0 = time.monotonic()

    def tick() -> None:
        t = time.monotonic() - t0
        # Moving target (finite diff -> smoothed velocity in node)
        tx = 28.0 - 3.5 * t
        ty = 6.0 - 0.4 * t
        tz = 22.0 - 0.9 * t
        pub_t.publish(Point(x=tx, y=ty, z=tz))
        pubs[0].publish(Point(x=-5.0, y=0.0, z=0.0))
        pubs[1].publish(Point(x=4.0, y=-4.0, z=0.0))
        pubs[2].publish(Point(x=-4.0, y=5.0, z=0.0))

    fake.create_timer(0.05, tick)
    tick()

    cmd = [
        'ros2',
        'run',
        'gazebo_target_sim',
        'interception_logic_node',
        '--ros-args',
        '-p',
        'dome_enabled:=false',
        '-p',
        'rate_hz:=20.0',
        '-p',
        'intercept_debug:=true',
        '-p',
        'log_period_s:=1.0',
        '-p',
        'closing_speed_m_s:=12.0',
        '-p',
        'max_speed_m_s:=14.0',
        '-p',
        'use_pn_refinement:=false',
        '-p',
        'pursuit_lead_blend:=0.28',
        '-p',
        'lock_selected_after_first:=false',
        '-p',
        'selection_margin_s:=0.3',
        '-p',
        'switch_window_s:=0.5',
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    end = time.monotonic() + 14.0
    buf: list[str] = []
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(fake, timeout_sec=0.05)
            if proc.stdout and proc.poll() is None:
                r, _, _ = select.select([proc.stdout], [], [], 0.0)
                if r:
                    line = proc.stdout.readline()
                    if line:
                        sys.stdout.write(line)
                        sys.stdout.flush()
                        buf.append(line)
            elif proc.poll() is not None:
                break
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        fake.destroy_node()
        rclpy.shutdown()

    text = ''.join(buf)
    if '[Intercept Debug]' not in text:
        print(
            '\n--- NOTE: No [Intercept Debug] captured (node may have exited early or no guidance tick). ---\n',
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
