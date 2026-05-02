"""Kinematic interceptor *drone* in Gazebo: set_pose + /interceptor/position; follows cmd_velocity."""

from __future__ import annotations

import math
import os
import queue
import shutil
import subprocess
import threading
from collections import deque
from typing import Sequence

import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from gazebo_target_sim.clock_reset import subscribe_sim_time_reset
from gazebo_target_sim.gz_pose_tools import fmt_pose_req_full, quat_align_body_x_to_velocity


def _gz_local_ip() -> str:
    """Return best local IP for gz transport; falls back to 127.0.0.1."""
    import socket as _s
    try:
        s = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        if not ip.startswith('127.') and not ip.startswith('169.254.'):
            return ip
    except Exception:
        pass
    return '127.0.0.1'


class InterceptorControllerNode(Node):
    """
    Integrates world velocity each step (default 10 Hz): ``p += v * dt``, then ``gz service set_pose``.

    Subscribes to ``geometry_msgs/Vector3`` on ``cmd_velocity_topic`` (m/s). If the command is stale
    (no message for ``cmd_timeout_s``), uses ``fallback_vel_*`` (default zero).

    **Counter-UAS drone:** spawns near **z = 0**. The SDF model is built with **forward = +X**;
    each tick we set **orientation** so body +X aligns with the velocity vector (nose into the
    intercept). Guidance from ``interception_logic_node`` remains unchanged (3-D pursuit to collide).
    """

    def __init__(self) -> None:
        super().__init__('interceptor_controller_node')
        self.declare_parameter('world_name', 'counter_uas_target')
        self.declare_parameter('model_name', 'interceptor')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('origin_x', -15.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_z', 0.0)
        self.declare_parameter('start_idle', True)
        self.declare_parameter('reset_on_sim_clock_rewind', True)
        self.declare_parameter('vel_x', 0.0)
        self.declare_parameter('vel_y', 0.0)
        self.declare_parameter('vel_z', 0.0)
        self.declare_parameter('cmd_velocity_topic', '/interceptor/cmd_velocity')
        self.declare_parameter('selected_id_topic', '/interceptor/selected_id')
        # Multi-target: non-empty ``assigned_target`` (e.g. target_0) arms motion; empty disarms.
        self.declare_parameter('assigned_target_topic', '')
        self.declare_parameter('cmd_timeout_s', 0.75)
        self.declare_parameter('max_speed_m_s', 8.0)
        self.declare_parameter('service_timeout_ms', 800)
        self.declare_parameter('position_topic', '/interceptor/position')
        self.declare_parameter('marker_topic', '/interceptor/marker')
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('marker_scale', 0.35)
        self.declare_parameter('log_period_s', 1.0)
        self.declare_parameter('use_vel_smoothing', True)
        self.declare_parameter('vel_smooth_alpha', 0.35)
        self.declare_parameter('orient_min_speed_m_s', 0.08)
        # ── Phase 5 autopilot/effector model ──────────────────────────────────────────
        # When ``autopilot.tau_s`` > 0 we replace the legacy EMA with a first-order
        # bandwidth model:  dv_applied/dt = (v_cmd_delayed - v_applied) / tau.
        # This produces a defensible velocity response curve: a measurable bandwidth
        # ``f3dB ~ 1 / (2*pi*tau)`` and a 63%-rise time equal to tau.  Defaults to 0
        # to preserve the existing EMA path.
        self.declare_parameter('autopilot.tau_s', 0.0)
        # Pure transport delay between receiving a command and acting on it; implemented as
        # a bounded FIFO sized by ``ceil(cmd_delay_s / dt)``.  Use to characterise how robust
        # the guidance loop is to extra plant delay on top of sensor latency.
        self.declare_parameter('autopilot.cmd_delay_s', 0.0)

        self._world = str(self.get_parameter('world_name').value).strip()
        self._model = str(self.get_parameter('model_name').value).strip()
        self._px = float(self.get_parameter('origin_x').value)
        self._py = float(self.get_parameter('origin_y').value)
        # Start on ground by default (z=0) unless explicitly overridden.
        self._pz = float(self.get_parameter('origin_z').value)
        self._fb_vx = float(self.get_parameter('vel_x').value)
        self._fb_vy = float(self.get_parameter('vel_y').value)
        self._fb_vz = float(self.get_parameter('vel_z').value)
        self._cmd_topic = str(self.get_parameter('cmd_velocity_topic').value).strip()
        self._sel_topic = str(self.get_parameter('selected_id_topic').value).strip()
        self._assigned_topic = str(self.get_parameter('assigned_target_topic').value).strip()
        self._cmd_timeout = max(float(self.get_parameter('cmd_timeout_s').value), 0.05)
        self._vmax = max(float(self.get_parameter('max_speed_m_s').value), 0.1)
        timeout_ms = int(self.get_parameter('service_timeout_ms').value)
        self._timeout_s = max(float(timeout_ms) * 1e-3, 0.05)
        pos_topic = str(self.get_parameter('position_topic').value).strip()
        self._marker_topic = str(self.get_parameter('marker_topic').value).strip()
        self._do_marker = bool(self.get_parameter('publish_marker').value)
        self._marker_scale = float(self.get_parameter('marker_scale').value)
        self._log_period = max(float(self.get_parameter('log_period_s').value), 0.5)
        self._idle = bool(self.get_parameter('start_idle').value)
        self._smooth = bool(self.get_parameter('use_vel_smoothing').value)
        self._salpha = min(max(float(self.get_parameter('vel_smooth_alpha').value), 0.05), 1.0)
        self._v_orient_floor = max(float(self.get_parameter('orient_min_speed_m_s').value), 1e-3)
        self._ap_tau_s = max(0.0, float(self.get_parameter('autopilot.tau_s').value))
        self._ap_delay_s = max(0.0, float(self.get_parameter('autopilot.cmd_delay_s').value))
        self._vx_s = 0.0
        self._vy_s = 0.0
        self._vz_s = 0.0
        # Command FIFO for the autopilot transport delay model (length 0 disables).
        self._cmd_buf: deque[tuple[float, float, float]] = deque()
        self._cmd_buf_len = 0  # set after _dt is known, below.

        self._cmd = Vector3()
        self._have_cmd = False
        self._cmd_stamp: Time = self.get_clock().now()
        self._last_log = self.get_clock().now()
        self._selected_id: str = ''
        self._assigned_target: str = ''

        rate = max(float(self.get_parameter('rate_hz').value), 0.5)
        self._dt = 1.0 / rate
        # Size the transport-delay FIFO so it matches ``cmd_delay_s`` at the configured rate.
        self._cmd_buf_len = int(math.ceil(self._ap_delay_s / self._dt)) if self._ap_delay_s > 0 else 0
        if self._cmd_buf_len > 0:
            zero = (0.0, 0.0, 0.0)
            for _ in range(self._cmd_buf_len):
                self._cmd_buf.append(zero)
        self._gz = shutil.which('gz')
        if not self._gz:
            self.get_logger().fatal('gz CLI not found in PATH.')
            raise RuntimeError('gz not found')

        self._service = f'/world/{self._world}/set_pose'
        self._warned_fail = False
        self._pub_pt = self.create_publisher(Point, pos_topic, 10)
        self._pub_mk = self.create_publisher(Marker, self._marker_topic, 10) if self._do_marker else None
        self.create_subscription(Vector3, self._cmd_topic, self._on_cmd, 10)
        self.create_subscription(String, self._sel_topic, self._on_selected, 10)
        if self._assigned_topic:
            self.create_subscription(String, self._assigned_topic, self._on_assigned_target, 10)

        # Background thread for set_pose — avoids blocking the ROS executor.
        self._pose_queue: queue.Queue = queue.Queue(maxsize=1)
        self._pose_worker = threading.Thread(target=self._pose_worker_loop, daemon=True)
        self._pose_worker.start()

        self._timer = self.create_timer(self._dt, self._on_timer)
        # Snap all drones to origin at startup so stale Gazebo positions from
        # previous runs don't leave models floating in the air.
        self.create_timer(1.5, self._reset_to_origin_once)
        if bool(self.get_parameter('reset_on_sim_clock_rewind').value):
            subscribe_sim_time_reset(self, self._on_gz_sim_reset)
        self.get_logger().info(
            f'Interceptor drone -> {self._service} model={self._model!r} at {rate:.1f} Hz; '
            f'cmd={self._cmd_topic} (timeout {self._cmd_timeout}s); idle={self._idle}; {pos_topic}',
        )

    def _reset_to_origin_once(self) -> None:
        """One-shot timer: snap Gazebo model to origin on startup to clear stale poses."""
        ox = float(self.get_parameter('origin_x').value)
        oy = float(self.get_parameter('origin_y').value)
        oz = float(self.get_parameter('origin_z').value)
        self._call_set_pose(ox, oy, oz, 0.0, 0.0, 0.0, 1.0)

    def _on_gz_sim_reset(self) -> None:
        self.get_logger().info(f'Sim reset (/clock rewind): {self._model!r} back to origin pose.')
        self._px = float(self.get_parameter('origin_x').value)
        self._py = float(self.get_parameter('origin_y').value)
        self._pz = float(self.get_parameter('origin_z').value)
        self._vx_s = self._vy_s = self._vz_s = 0.0
        self._idle = bool(self.get_parameter('start_idle').value)
        self._cmd = Vector3()
        self._have_cmd = False
        self._selected_id = ''
        self._assigned_target = ''
        self._cmd_stamp = self.get_clock().now()
        # Physically move the Gazebo model back to origin so it doesn't float.
        self._call_set_pose(self._px, self._py, self._pz, 0.0, 0.0, 0.0, 1.0)

    def _on_cmd(self, msg: Vector3) -> None:
        self._cmd = msg
        self._have_cmd = True
        self._cmd_stamp = self.get_clock().now()

    def _on_selected(self, msg: String) -> None:
        if self._assigned_topic:
            return
        new_sel = str(msg.data).strip()
        self._selected_id = new_sel
        if self._idle and new_sel == self._model:
            # Launch: only this interceptor is selected.
            self._idle = False
            print(f'[LAUNCH] {self._model}', flush=True)
        elif not self._idle and not new_sel:
            # Empty selected_id: interception clears selection on HIT, outside dome, lost
            # target, etc. Do NOT treat all of these as "kamikaze remove" — that was deleting
            # the model when we only meant to stand down, and broke Gazebo pose updates.
            self._idle = True
            self._vx_s = self._vy_s = self._vz_s = 0.0
            ox = float(self.get_parameter('origin_x').value)
            oy = float(self.get_parameter('origin_y').value)
            oz = float(self.get_parameter('origin_z').value)
            self._px, self._py, self._pz = ox, oy, oz
            self._call_set_pose(ox, oy, oz, 0.0, 0.0, 0.0, 1.0)
            print(f'[STANDBY] {self._model} (selection cleared)', flush=True)
        elif not self._idle and new_sel != self._model:
            # A different interceptor was selected — stand down and return to ground.
            self._idle = True
            self._vx_s = self._vy_s = self._vz_s = 0.0
            ox = float(self.get_parameter('origin_x').value)
            oy = float(self.get_parameter('origin_y').value)
            oz = float(self.get_parameter('origin_z').value)
            self._px, self._py, self._pz = ox, oy, oz
            self._call_set_pose(ox, oy, oz, 0.0, 0.0, 0.0, 1.0)
            print(f'[STANDBY] {self._model}', flush=True)

    def _on_assigned_target(self, msg: String) -> None:
        if not self._assigned_topic:
            return
        self._assigned_target = str(msg.data).strip()
        if self._assigned_target:
            if self._idle:
                self._idle = False
                print(f'[LAUNCH] {self._model} -> {self._assigned_target}', flush=True)
        else:
            self._idle = True

    @staticmethod
    def _norm3(vx: float, vy: float, vz: float) -> float:
        return math.sqrt(vx * vx + vy * vy + vz * vz)

    def _applied_vel(self) -> tuple[float, float, float]:
        if self._assigned_topic and not self._assigned_target:
            return (0.0, 0.0, 0.0)
        if self._idle:
            return (0.0, 0.0, 0.0)
        now = self.get_clock().now()
        age = (now - self._cmd_stamp).nanoseconds * 1e-9
        if self._have_cmd and age <= self._cmd_timeout:
            vx, vy, vz = float(self._cmd.x), float(self._cmd.y), float(self._cmd.z)
        else:
            vx, vy, vz = self._fb_vx, self._fb_vy, self._fb_vz
        n = self._norm3(vx, vy, vz)
        if n > self._vmax and n > 1e-9:
            s = self._vmax / n
            vx, vy, vz = vx * s, vy * s, vz * s
        return (vx, vy, vz)

    def _smooth_vel(self, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
        if self._ap_tau_s > 0.0:
            # First-order autopilot bandwidth: x_{k+1} = x_k + (dt/tau) * (cmd - x_k).
            # Equivalent to an EMA with alpha = dt/(dt+tau); deriving alpha from tau lets the
            # response curve be specified by a single physically meaningful number.
            tau = self._ap_tau_s
            a = self._dt / (self._dt + tau)
            self._vx_s = self._vx_s + a * (vx - self._vx_s)
            self._vy_s = self._vy_s + a * (vy - self._vy_s)
            self._vz_s = self._vz_s + a * (vz - self._vz_s)
            return (self._vx_s, self._vy_s, self._vz_s)
        if not self._smooth:
            return (vx, vy, vz)
        a = self._salpha
        self._vx_s = a * vx + (1.0 - a) * self._vx_s
        self._vy_s = a * vy + (1.0 - a) * self._vy_s
        self._vz_s = a * vz + (1.0 - a) * self._vz_s
        return (self._vx_s, self._vy_s, self._vz_s)

    def _apply_cmd_delay(self, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
        """Push current command into a fixed-length FIFO and pop the delayed sample to act on.

        With ``autopilot.cmd_delay_s = 0`` the FIFO is empty and we return the input unchanged.
        At each tick we add the freshly received command and consume the oldest, so the consumer
        sees a delay of ``cmd_buf_len * dt`` seconds.
        """
        if self._cmd_buf_len <= 0:
            return (vx, vy, vz)
        self._cmd_buf.append((vx, vy, vz))
        return self._cmd_buf.popleft()

    def _quat_from_motion(self, vx: float, vy: float, vz: float, idle: bool) -> tuple[float, float, float, float]:
        if idle or self._norm3(vx, vy, vz) < self._v_orient_floor:
            return (0.0, 0.0, 0.0, 1.0)
        return quat_align_body_x_to_velocity(vx, vy, vz)

    def _pose_worker_loop(self) -> None:
        """Background thread: consumes pose requests and calls gz service without blocking ROS."""
        gz_env = os.environ.copy()
        if not gz_env.get('GZ_IP'):
            gz_env['GZ_IP'] = _gz_local_ip()
        while True:
            item = self._pose_queue.get()
            if item is None:
                break
            cmd = item
            try:
                subprocess.run(
                    cmd,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=self._timeout_s + 0.25,
                    env=gz_env,
                )
            except subprocess.TimeoutExpired:
                pass
            except Exception:
                pass

    def _call_set_pose(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> bool:
        req = fmt_pose_req_full(self._model, x, y, z, qx, qy, qz, qw)
        cmd: Sequence[str] = (
            self._gz,
            'service',
            '-s',
            self._service,
            '--reqtype',
            'gz.msgs.Pose',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            str(int(self._timeout_s * 1000)),
            '--req',
            req,
        )
        # Non-blocking: drop into the background queue (drop if full to keep latency low).
        try:
            self._pose_queue.put_nowait(cmd)
        except queue.Full:
            pass
        return True

    def _publish_marker(
        self,
        stamp,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        if self._pub_mk is None:
            return
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'map'
        m.ns = 'interceptor'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
        s = max(self._marker_scale, 0.05)
        m.scale.x = s * 1.35
        m.scale.y = s * 1.35
        m.scale.z = s * 0.38
        m.color = ColorRGBA(r=0.15, g=0.35, b=1.0, a=1.0)
        self._pub_mk.publish(m)

    def _maybe_log(self, vx: float, vy: float, vz: float) -> None:
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds * 1e-9 < self._log_period:
            return
        self._last_log = now
        age = (now - self._cmd_stamp).nanoseconds * 1e-9
        src = 'cmd' if self._have_cmd and age <= self._cmd_timeout else 'fallback'
        print(
            f'[interceptor_ctrl] pos=({self._px:.2f},{self._py:.2f},{self._pz:.2f}) '
            f'v_applied=({vx:.2f},{vy:.2f},{vz:.2f}) [{src}] dt={self._dt:.4f}s idle={self._idle}',
            flush=True,
        )

    def _on_timer(self) -> None:
        vx, vy, vz = self._applied_vel()
        idle = self._idle
        if self._assigned_topic and not self._assigned_target:
            idle = True
        if idle:
            self._vx_s = self._vy_s = self._vz_s = 0.0
            # Reset the autopilot transport buffer too so the next launch starts cleanly.
            if self._cmd_buf_len > 0:
                self._cmd_buf.clear()
                zero = (0.0, 0.0, 0.0)
                for _ in range(self._cmd_buf_len):
                    self._cmd_buf.append(zero)
            # Skip set_pose while idle — avoids Gazebo jitter on non-selected interceptors.
            p = Point()
            p.x, p.y, p.z = float(self._px), float(self._py), float(self._pz)
            self._pub_pt.publish(p)
            self._maybe_log(vx, vy, vz)
            return
        vx, vy, vz = self._apply_cmd_delay(vx, vy, vz)
        vx, vy, vz = self._smooth_vel(vx, vy, vz)
        self._px += vx * self._dt
        self._py += vy * self._dt
        self._pz += vz * self._dt
        qx, qy, qz, qw = self._quat_from_motion(vx, vy, vz, idle)
        self._call_set_pose(self._px, self._py, self._pz, qx, qy, qz, qw)
        p = Point()
        p.x, p.y, p.z = float(self._px), float(self._py), float(self._pz)
        self._pub_pt.publish(p)
        self._publish_marker(
            self.get_clock().now().to_msg(), self._px, self._py, self._pz, qx, qy, qz, qw,
        )
        self._maybe_log(vx, vy, vz)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InterceptorControllerNode()
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
