#!/usr/bin/env python3
"""
Smoke: interception_logic_node + synthetic /drone/position + interceptor positions.
No Gazebo. Asserts at least one ImpactEvent on /interception/impact_event within timeout.
"""

from __future__ import annotations

import os
import select
import subprocess
import sys
import time

import math
import rclpy
from gazebo_target_sim_interfaces.msg import ImpactEvent
from geometry_msgs.msg import Point
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class SyntheticWorld(Node):
    """Publishes a closing geometry until interception_logic declares HIT + ImpactEvent."""

    def __init__(self, done_event: dict[str, bool | str]) -> None:
        super().__init__('smoke_impact_world')
        self._done_event = done_event
        self._pub_t = self.create_publisher(Point, '/drone/position', 10)
        self._pub_i = [
            self.create_publisher(Point, f'/interceptor_{k}/position', 10)
            for k in range(3)
        ]
        self._subs = [
            self.create_subscription(ImpactEvent, '/interception/impact_event', self._on_impact, 10),
        ]

        tx, ty, tz = 220.0, 0.0, 140.0
        self._t = [tx, ty, tz]
        self._i0_start = [-280.0, 0.0, 132.0]
        self._i0 = list(self._i0_start)
        # interceptor_1/2 deliberately far worse TTI/first assign than interceptor_0 (the mover).
        self._i1 = [1850.0, -1600.0, 95.0]
        self._i2 = [-1200.0, 1650.0, 92.0]
        closing = 78.0
        ux, uy, uz = tx - self._i0[0], ty - self._i0[1], tz - self._i0[2]
        nu = math.sqrt(ux * ux + uy * uy + uz * uz)
        s = closing / max(nu, 1e-6)
        self._vi0 = (ux * s, uy * s, uz * s)
        self._t0_wall = time.monotonic()
        self.create_timer(0.02, self._tick)
        self.get_logger().info('Synthetic hostile close; waiting for ImpactEvent…')

    def _on_impact(self, msg: ImpactEvent) -> None:
        if self._done_event.get('impact'):
            return
        self.get_logger().info(
            f'ImpactEvent interceptor={msg.interceptor_id!r} target_label={msg.target_label!r}',
        )
        self._done_event['impact'] = True
        self._done_event['interceptor'] = msg.interceptor_id

    def _tick(self) -> None:
        if self._done_event.get('impact'):
            return
        dt = 0.02
        self._i0[0] += self._vi0[0] * dt
        self._i0[1] += self._vi0[1] * dt
        self._i0[2] += self._vi0[2] * dt
        p = Point()
        p.x, p.y, p.z = float(self._t[0]), float(self._t[1]), float(self._t[2])
        self._pub_t.publish(p)
        for idx, arr in enumerate((self._i0, self._i1, self._i2)):
            q = Point()
            q.x, q.y, q.z = float(arr[0]), float(arr[1]), float(arr[2])
            self._pub_i[idx].publish(q)

        elapsed = time.monotonic() - self._t0_wall
        if elapsed > 60.0 and not self._done_event.get('impact'):
            self.get_logger().warning('still no ImpactEvent after 60s')


def main() -> int:
    if not os.environ.get('AMENT_PREFIX_PATH'):
        print('Source install/setup.bash first.', file=sys.stderr)
        return 1

    rclpy.init()
    done: dict[str, bool | str] = {'impact': False}
    synth = SyntheticWorld(done)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(synth)

    cmd = [
        'ros2',
        'run',
        'gazebo_target_sim',
        'interception_logic_node',
        '--ros-args',
        '-p',
        'dome_enabled:=false',
        '-p',
        'rate_hz:=40.0',
        '-p',
        'intercept_debug:=false',
        '-p',
        'intercept_measurement_source:=ground_truth',
        '-p',
        'target_topic:=/drone/position',
        '-p',
        'publish_impact_event:=true',
        '-p',
        'impact_event_topic:=/interception/impact_event',
        '-p',
        'publish_hit_markers:=false',
        '-p',
        'publish_intercept_markers:=false',
        '-p',
        'lock_selected_after_first:=true',
        '-p',
        'lock_engaged_interceptor_until_hit:=true',
        '-p',
        'hit_threshold_m:=2.5',
        '-p',
        'hit_min_interceptor_z_m:=0.05',
        '-p',
        'hit_min_interceptor_travel_m:=1.5',
        '-p',
        'hit_min_target_z_m:=0.1',
        '-p',
        'closing_speed_m_s:=82.0',
        '-p',
        'max_speed_m_s:=88.0',
        '-p',
        'max_intercept_time_s:=18.0',
        '-p',
        'min_intercept_time_s:=0.03',
        '-p',
        'use_pn_refinement:=false',
        '-p',
        'pursuit_lead_blend:=0.22',
        '-p',
        'stop_signal_repeat_duration_s:=2.0',
        '-p',
        'classification_gating_enabled:=false',
        '-p',
        'feasibility_based_engagement:=false',
        '-p',
        'intercept_mc_use_light_hit_model:=true',
    ]

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    end = time.monotonic() + 55.0
    hit_line = ''
    buf: list[str] = []
    try:
        assert proc.stdout is not None
        while time.monotonic() < end and rclpy.ok():
            if done.get('impact'):
                print('PASSED: received ImpactEvent', flush=True)
                return 0
            ex.spin_once(timeout_sec=0.06)
            r, _, _ = select.select([proc.stdout], [], [], 0.0)
            if r:
                ln = proc.stdout.readline()
                if ln:
                    buf.append(ln)
                    sys.stdout.write(ln)
                    sys.stdout.flush()
                    if '[HIT]' in ln:
                        hit_line = ln.strip()
                    if ln.startswith('terminate') or ln.startswith('Failed'):
                        break
            if proc.poll() is not None:
                sys.stdout.write('--- interception_logic exited ---\n')
                sys.stdout.flush()
                break
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            proc.kill()

        synth.destroy_node()
        rclpy.shutdown()

    if done.get('impact'):
        return 0
    print('\nFAILED: no ImpactEvent before timeout', file=sys.stderr)
    if hit_line:
        print(f'NOTE: saw HIT stdout but missed DDS?: {hit_line}', file=sys.stderr)
    tail = ''.join(buf[-35:])
    if tail:
        print(f'--- last stdout/stderr lines ---\n{tail}', file=sys.stderr)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
