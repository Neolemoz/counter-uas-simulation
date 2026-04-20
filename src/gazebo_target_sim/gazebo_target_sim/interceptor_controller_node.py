"""Kinematic interceptor in Gazebo: set_pose + /interceptor/position; follows /interceptor/cmd_velocity."""

from __future__ import annotations

import math
import shutil
import subprocess
from typing import Sequence

import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from gazebo_target_sim.gz_pose_tools import fmt_pose_req


class InterceptorControllerNode(Node):
    """
    Integrates world velocity each step (default 10 Hz): ``p += v * dt``, then ``gz service set_pose``.

    Subscribes to ``geometry_msgs/Vector3`` on ``cmd_velocity_topic`` (m/s). If the command is stale
    (no message for ``cmd_timeout_s``), uses ``fallback_vel_*`` (default zero).
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
        self.declare_parameter('vel_x', 0.0)
        self.declare_parameter('vel_y', 0.0)
        self.declare_parameter('vel_z', 0.0)
        self.declare_parameter('cmd_velocity_topic', '/interceptor/cmd_velocity')
        self.declare_parameter('selected_id_topic', '/interceptor/selected_id')
        self.declare_parameter('cmd_timeout_s', 0.75)
        self.declare_parameter('max_speed_m_s', 8.0)
        self.declare_parameter('service_timeout_ms', 800)
        self.declare_parameter('position_topic', '/interceptor/position')
        self.declare_parameter('marker_topic', '/interceptor/marker')
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('marker_scale', 0.35)
        self.declare_parameter('log_period_s', 1.0)

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

        self._cmd = Vector3()
        self._have_cmd = False
        self._cmd_stamp: Time = self.get_clock().now()
        self._last_log = self.get_clock().now()
        self._selected_id: str = ''

        rate = max(float(self.get_parameter('rate_hz').value), 0.5)
        self._dt = 1.0 / rate
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

        self._timer = self.create_timer(self._dt, self._on_timer)
        self.get_logger().info(
            f'Interceptor -> {self._service} model={self._model!r} at {rate:.1f} Hz; '
            f'cmd={self._cmd_topic} (timeout {self._cmd_timeout}s); idle={self._idle}; {pos_topic}',
        )

    def _on_cmd(self, msg: Vector3) -> None:
        self._cmd = msg
        self._have_cmd = True
        self._cmd_stamp = self.get_clock().now()

    def _on_selected(self, msg: String) -> None:
        self._selected_id = str(msg.data).strip()
        if self._idle and self._selected_id == self._model:
            # Launch is gated ONLY by selection (not cmd magnitude).
            self._idle = False
            print(f'[LAUNCH] {self._model}', flush=True)

    @staticmethod
    def _norm3(vx: float, vy: float, vz: float) -> float:
        return math.sqrt(vx * vx + vy * vy + vz * vz)

    def _applied_vel(self) -> tuple[float, float, float]:
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

    def _call_set_pose(self, x: float, y: float, z: float) -> bool:
        req = fmt_pose_req(self._model, x, y, z)
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
        try:
            r = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=self._timeout_s + 0.25,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warning('set_pose subprocess timed out')
            return False
        if r.returncode != 0:
            if not self._warned_fail:
                self.get_logger().warning(
                    f'set_pose failed rc={r.returncode} stderr={r.stderr!r}',
                )
                self._warned_fail = True
            return False
        self._warned_fail = False
        return True

    def _publish_marker(self, stamp, x: float, y: float, z: float) -> None:
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
        m.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        s = max(self._marker_scale, 0.05)
        m.scale.x = s
        m.scale.y = s
        m.scale.z = s * 0.6
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
            f'v_applied=({vx:.2f},{vy:.2f},{vz:.2f}) [{src}]',
            flush=True,
        )

    def _on_timer(self) -> None:
        vx, vy, vz = self._applied_vel()
        self._px += vx * self._dt
        self._py += vy * self._dt
        self._pz += vz * self._dt
        self._call_set_pose(self._px, self._py, self._pz)
        p = Point()
        p.x, p.y, p.z = float(self._px), float(self._py), float(self._pz)
        self._pub_pt.publish(p)
        self._publish_marker(self.get_clock().now().to_msg(), self._px, self._py, self._pz)
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
