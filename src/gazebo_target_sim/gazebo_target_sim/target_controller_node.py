"""Drive Gazebo target pose via /world/<world>/set_pose (gz.msgs.Pose); attack path toward asset + descent."""

from __future__ import annotations

import math
import os
from collections import deque
import shutil
import subprocess
from typing import Sequence

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, ColorRGBA

from gazebo_target_sim.clock_reset import subscribe_sim_time_reset
from gazebo_target_sim.gz_entity_tools import fmt_remove_model_req, fmt_spawn_model_from_file
from gazebo_target_sim.gz_pose_tools import fmt_pose_req


class TargetControllerNode(Node):
    """
    Calls ``gz service -s /world/<world>/set_pose`` at fixed rate (default 10 Hz).

    Counter-UAS hostile track: optional **orbit** in XY (optionally slow **spiral descent**), then
    **attack** either **LOS toward (0,0,0)** at ``los_closing_speed_m_s`` (slanted 3D path) or
    decoupled XY + ``dive_speed``.
    """

    def __init__(self) -> None:
        super().__init__('target_controller_node')
        self.declare_parameter('world_name', 'counter_uas_target')
        self.declare_parameter('model_name', 'sphere_target')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('start_x_m', -35.0)
        self.declare_parameter('start_y_m', 0.0)
        self.declare_parameter('start_z_m', 38.0)
        self.declare_parameter('t_dive_s', 4.0)
        self.declare_parameter('approach_speed', 8.0)
        # Negative = downward: intruder comes from altitude toward the defended volume.
        self.declare_parameter('approach_vz', -3.5)
        self.declare_parameter('dive_speed', -7.0)
        self.declare_parameter('log_period_s', 1.0)
        self.declare_parameter('service_timeout_ms', 800)
        self.declare_parameter('publish_drone_position', True)
        self.declare_parameter('drone_position_topic', '/drone/position')
        self.declare_parameter('stop_topic', '/target/stop')
        # หลัง STOP (โหมดไม่ระเบิด): เรียก set_pose ถี่ขึ้นเพื่อดับความเร็วค้างจากแรงปะทะ
        self.declare_parameter('hold_pose_rate_hz', 100.0)
        # ชนแล้วลบ sphere_target และ spawn วัตถุมองเห็นระเบิดสั้นๆ (UserCommands remove/create)
        self.declare_parameter('explode_on_hit', True)
        self.declare_parameter('explosion_fade_s', 4.0)
        self.declare_parameter('target_debug_log_enabled', True)
        self.declare_parameter('target_debug_marker_enabled', True)
        self.declare_parameter('target_debug_marker_topic', '/target_debug_marker')
        self.declare_parameter('target_debug_marker_frame_id', 'map')
        self.declare_parameter('target_debug_marker_scale', 0.55)
        self.declare_parameter('target_debug_trail_enabled', True)
        self.declare_parameter('target_debug_trail_max_points', 400)
        self.declare_parameter('target_debug_trajectory_topic', '/target_debug_trajectory')
        self.declare_parameter('target_debug_trail_line_width', 0.12)
        # Optional: one horizontal orbit in XY (fixed z), then existing attack toward (0,0) + dive.
        self.declare_parameter('orbit_enabled', False)
        self.declare_parameter('orbit_turns', 1.0)
        self.declare_parameter('orbit_duration_s', 22.0)
        self.declare_parameter('orbit_z_hold', True)
        # If ``orbit_z_hold`` is False: vertical rate during orbit (m/s, <= 0). If 0, uses ``dive_speed``.
        self.declare_parameter('orbit_descent_m_s', 0.0)
        self.declare_parameter('attack_xy_snap_m', 0.45)
        # True: single 3D closing velocity toward (0,0,0) — slanted descent (not vertical-then-drop).
        self.declare_parameter('attack_los_to_origin', True)
        # <= 0: use hypot(|approach_speed|, |dive_speed|)
        self.declare_parameter('los_closing_speed_m_s', 0.0)

        self._world = str(self.get_parameter('world_name').value).strip()
        self._model = str(self.get_parameter('model_name').value).strip()
        self._px = float(self.get_parameter('start_x_m').value)
        self._py = float(self.get_parameter('start_y_m').value)
        self._pz = float(self.get_parameter('start_z_m').value)
        self._t_dive = max(float(self.get_parameter('t_dive_s').value), 0.0)
        self._v_approach = float(self.get_parameter('approach_speed').value)
        self._v_approach_z = min(float(self.get_parameter('approach_vz').value), 0.0)
        self._v_dive_z = float(self.get_parameter('dive_speed').value)
        self._log_period = max(float(self.get_parameter('log_period_s').value), 0.2)
        timeout_ms = int(self.get_parameter('service_timeout_ms').value)
        self._timeout_s = max(float(timeout_ms) * 1e-3, 0.05)
        self._pub_gt = bool(self.get_parameter('publish_drone_position').value)
        gt_topic = str(self.get_parameter('drone_position_topic').value).strip()
        self._stop_topic = str(self.get_parameter('stop_topic').value).strip()
        self._stopped = False
        self._hold_hz = max(float(self.get_parameter('hold_pose_rate_hz').value), 10.0)
        self._explode_on_hit = bool(self.get_parameter('explode_on_hit').value)
        self._explosion_fade_s = max(float(self.get_parameter('explosion_fade_s').value), 0.2)
        self._explosion_fx_name: str | None = None
        self._explosion_fade_timer = None

        self._debug_log = bool(self.get_parameter('target_debug_log_enabled').value)
        self._debug_marker_on = bool(self.get_parameter('target_debug_marker_enabled').value)
        self._debug_marker_topic = str(self.get_parameter('target_debug_marker_topic').value).strip()
        self._debug_marker_frame = str(self.get_parameter('target_debug_marker_frame_id').value).strip() or 'map'
        self._debug_marker_scale = max(float(self.get_parameter('target_debug_marker_scale').value), 0.05)
        self._trail_on = bool(self.get_parameter('target_debug_trail_enabled').value)
        self._trail_max = max(int(self.get_parameter('target_debug_trail_max_points').value), 8)
        self._traj_topic = str(self.get_parameter('target_debug_trajectory_topic').value).strip()
        self._trail_line_w = max(float(self.get_parameter('target_debug_trail_line_width').value), 0.01)
        self._trail: deque[Point] = deque(maxlen=self._trail_max)

        self._orbit_enabled = bool(self.get_parameter('orbit_enabled').value)
        self._orbit_turns = max(float(self.get_parameter('orbit_turns').value), 0.1)
        self._orbit_duration_s = max(float(self.get_parameter('orbit_duration_s').value), 0.5)
        self._orbit_z_hold = bool(self.get_parameter('orbit_z_hold').value)
        _od = float(self.get_parameter('orbit_descent_m_s').value)
        self._orbit_vz = min(0.0, _od) if abs(_od) > 1e-9 else float(self._v_dive_z)
        self._snap_m = max(float(self.get_parameter('attack_xy_snap_m').value), 0.0)
        self._attack_los = bool(self.get_parameter('attack_los_to_origin').value)
        _los = float(self.get_parameter('los_closing_speed_m_s').value)
        if _los <= 0.0:
            _los = math.hypot(abs(self._v_approach), abs(self._v_dive_z))
        self._los_speed = max(_los, 0.05)
        self._recompute_orbit_frame()

        share = get_package_share_directory('gazebo_target_sim')
        self._explosion_sdf_path = os.path.join(share, 'models', 'hit_explosion', 'model.sdf')
        if self._explode_on_hit and not os.path.isfile(self._explosion_sdf_path):
            self.get_logger().warning(
                f'explode_on_hit: SDF not found at {self._explosion_sdf_path} (install package share).',
            )

        rate = max(float(self.get_parameter('rate_hz').value), 0.5)
        self._motion_rate_hz = rate
        self._gz = shutil.which('gz')
        if not self._gz:
            self.get_logger().fatal('gz CLI not found in PATH; install Gazebo / gz-sim.')
            raise RuntimeError('gz not found')

        self._service = f'/world/{self._world}/set_pose'
        self._t0 = self.get_clock().now()
        self._last_tick_time: Time | None = None
        self._last_log_time: Time | None = None
        self._warned_fail = False
        self._pub_point = self.create_publisher(Point, gt_topic, 10) if self._pub_gt else None
        self._pub_debug_marker = (
            self.create_publisher(Marker, self._debug_marker_topic, 10) if self._debug_marker_on else None
        )
        self._pub_traj = (
            self.create_publisher(Marker, self._traj_topic, 10)
            if (self._debug_marker_on and self._trail_on)
            else None
        )
        self._reset_trail()
        _sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Bool, self._stop_topic, self._on_stop, _sub_qos)
        self._timer_period_s = 1.0 / rate
        self._timer = self.create_timer(self._timer_period_s, self._on_timer)
        self._motion_rate_hz = rate
        subscribe_sim_time_reset(self, self._on_gz_sim_reset)
        extra = f' + {gt_topic} (ground truth)' if self._pub_gt else ''
        orbit_note = ''
        if self._orbit_enabled and self._orbit_r >= 0.05:
            if self._orbit_z_hold:
                orbit_note = f' orbit: {self._orbit_turns:g} turn(s) @ z≈{self._orbit_z_start:.1f} m;'
            else:
                orbit_note = (
                    f' orbit: {self._orbit_turns:g} turn(s) spiral vz={self._orbit_vz:.3g} m/s;'
                )
        los_note = (
            f' LOS attack closing={self._los_speed:.3g} m/s → (0,0,0);'
            if self._attack_los
            else f' decoupled attack vz={self._v_dive_z} m/s;'
        )
        self.get_logger().info(
            f'Target pose -> {self._service} model={self._model!r} at {rate:.1f} Hz '
            f'{orbit_note}{los_note} start=({self._px},{self._py},{self._pz}){extra}',
        )
        self.get_logger().info(
            'Visual: there is no mesh "drone" in this world — the target is the orange sphere '
            f'({self._model!r}); interceptors are the small colored boxes.',
        )
        self.get_logger().info(
            f'On hit: explode_on_hit={self._explode_on_hit} (remove target + short explosion visual).',
        )
        self.get_logger().info(
            'Gazebo GUI Reset: require ros_gz_bridge /clock + topic /world/<name>/clock (see launch).',
        )

    def _on_gz_sim_reset(self) -> None:
        """gz กด reset -> เวลาจำลองย้อน — รีเซ็บตำแหน่ง/เฟสให้ตรงพารามิเตอร์เริ่มต้น."""
        self.get_logger().info('Sim reset (/clock rewind): re-arm target trajectory and timers.')
        self._stopped = False
        self._px = float(self.get_parameter('start_x_m').value)
        self._py = float(self.get_parameter('start_y_m').value)
        self._pz = float(self.get_parameter('start_z_m').value)
        if self._explosion_fade_timer is not None:
            self.destroy_timer(self._explosion_fade_timer)
            self._explosion_fade_timer = None
        self._explosion_fx_name = None
        if self._timer is None:
            self._timer_period_s = 1.0 / self._motion_rate_hz
            self._timer = self.create_timer(self._timer_period_s, self._on_timer)
        self._t0 = self.get_clock().now()
        self._last_tick_time = None
        self._last_log_time = None
        self._reset_trail()
        self._recompute_orbit_frame()

    def _recompute_orbit_frame(self) -> None:
        self._orbit_r = math.hypot(self._px, self._py)
        self._theta0 = math.atan2(self._py, self._px)
        self._orbit_z_start = float(self._pz)
        self._phase = 'orbit' if self._orbit_enabled and self._orbit_r >= 0.05 else 'attack'

    def _reset_trail(self) -> None:
        self._trail.clear()

    def _on_stop(self, msg: Bool) -> None:
        if self._stopped:
            return
        if not bool(msg.data):
            return
        self._stopped = True
        if self._explode_on_hit:
            self._begin_explosion_sequence()
            return
        self.get_logger().info(
            f'Target STOP: hold pose @ {self._hold_hz:.0f} Hz (was {1.0 / self._timer_period_s:.1f} Hz).',
        )
        if self._timer is not None:
            self.destroy_timer(self._timer)
        self._timer_period_s = 1.0 / self._hold_hz
        self._timer = self.create_timer(self._timer_period_s, self._on_timer)

    def _begin_explosion_sequence(self) -> None:
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._explosion_fade_timer is not None:
            self.destroy_timer(self._explosion_fade_timer)
            self._explosion_fade_timer = None

        ex, ey, ez = float(self._px), float(self._py), float(self._pz)
        if self._pub_point is not None:
            p = Point()
            p.x, p.y, p.z = ex, ey, ez
            self._pub_point.publish(p)

        ok_rm = self._gz_world_service('remove', 'gz.msgs.Entity', fmt_remove_model_req(self._model))
        if not ok_rm:
            ok_rm = self._gz_world_service('remove', 'gz.msgs.Entity', fmt_remove_model_req(self._model))
        if not ok_rm:
            self.get_logger().warning(f'gz remove {self._model!r} failed (model may already be gone).')

        self._explosion_fx_name = f'hit_exp_{self.get_clock().now().nanoseconds}'
        ok_sp = False
        if os.path.isfile(self._explosion_sdf_path):
            ok_sp = self._gz_world_service(
                'create',
                'gz.msgs.EntityFactory',
                fmt_spawn_model_from_file(self._explosion_sdf_path, self._explosion_fx_name, ex, ey, ez),
            )
        if not ok_sp:
            self.get_logger().warning(
                'gz create explosion visual failed (check /world/.../create and SDF path).',
            )

        self.get_logger().info(
            f'Impact sequence at ({ex:.2f},{ey:.2f},{ez:.2f}); fx={self._explosion_fx_name!r}',
        )
        self._explosion_fade_timer = self.create_timer(
            float(self._explosion_fade_s),
            self._on_explosion_fade,
        )

    def _on_explosion_fade(self) -> None:
        if self._explosion_fade_timer is not None:
            self.destroy_timer(self._explosion_fade_timer)
            self._explosion_fade_timer = None
        name = self._explosion_fx_name
        self._explosion_fx_name = None
        if name:
            self._gz_world_service('remove', 'gz.msgs.Entity', fmt_remove_model_req(name))

    def _gz_world_service(self, service_suffix: str, reqtype: str, req: str) -> bool:
        svc = f'/world/{self._world}/{service_suffix}'
        cmd: Sequence[str] = (
            self._gz,
            'service',
            '-s',
            svc,
            '--reqtype',
            reqtype,
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
            self.get_logger().warning(f'gz service {svc!r} timed out')
            return False
        if r.returncode != 0:
            self.get_logger().warning(f'gz service {svc!r} rc={r.returncode} stderr={r.stderr!r}')
            return False
        return True

    def _snap_attack_pose_if_needed(self) -> None:
        if self._snap_m <= 0.0:
            return
        if self._attack_los:
            n3 = math.sqrt(self._px * self._px + self._py * self._py + self._pz * self._pz)
            if n3 < self._snap_m:
                self._px = self._py = self._pz = 0.0
        else:
            if math.hypot(self._px, self._py) < self._snap_m:
                self._px = self._py = 0.0

    def _velocity(self, t_elapsed_s: float) -> tuple[float, float, float]:
        """Attack phase: LOS toward (0,0,0) or decoupled XY + ``dive_speed``. ``t_elapsed_s`` unused."""
        del t_elapsed_s  # kept for call-site compatibility
        eps = max(1e-6, self._snap_m)
        if self._attack_los:
            dx, dy, dz = -self._px, -self._py, -self._pz
            n = math.sqrt(dx * dx + dy * dy + dz * dz)
            if n < eps:
                return (0.0, 0.0, 0.0)
            inv = self._los_speed / n
            return (dx * inv, dy * inv, dz * inv)
        dx = -self._px
        dy = -self._py
        norm = math.hypot(dx, dy)
        v_h = float(self._v_approach)
        v_h_abs = abs(v_h)
        if norm < eps:
            vx, vy = 0.0, 0.0
        else:
            inv = 1.0 / norm
            vx = dx * inv * v_h
            vy = dy * inv * v_h
        vz = float(self._v_dive_z)
        h = math.hypot(vx, vy)
        if h > 1e-12 and v_h_abs > 1e-12 and h > v_h_abs + 1e-9:
            s = v_h_abs / h
            vx, vy = vx * s, vy * s
        return (vx, vy, vz)

    def _publish_target_debug_marker(self) -> None:
        if self._pub_debug_marker is None:
            return
        stamp = self.get_clock().now().to_msg()
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self._debug_marker_frame
        m.ns = 'target_debug'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(self._px)
        m.pose.position.y = float(self._py)
        m.pose.position.z = float(self._pz)
        m.pose.orientation.w = 1.0
        s = self._debug_marker_scale
        m.scale.x = s
        m.scale.y = s
        m.scale.z = s
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.95)
        m.lifetime.sec = 0
        self._pub_debug_marker.publish(m)
        self._publish_target_debug_trajectory()

    def _publish_target_debug_trajectory(self) -> None:
        if self._pub_traj is None or len(self._trail) < 2:
            return
        stamp = self.get_clock().now().to_msg()
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self._debug_marker_frame
        m.ns = 'target_debug'
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self._trail_line_w
        m.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.9)
        m.points = list(self._trail)
        m.lifetime.sec = 0
        self._pub_traj.publish(m)

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
                    f'set_pose failed rc={r.returncode} stderr={r.stderr!r} stdout={r.stdout!r}',
                )
                self._warned_fail = True
            return False
        self._warned_fail = False
        return True

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        # โหมดระเบิด: ไม่มี timer นี้หลังชน (ลบโมเดลแล้ว)
        if self._stopped and self._explode_on_hit:
            return
        # หลัง STOP (ไม่ระเบิด): ค้างด้วย set_pose ซ้ำ
        if self._stopped:
            if self._pub_point is not None:
                p = Point()
                p.x, p.y, p.z = float(self._px), float(self._py), float(self._pz)
                self._pub_point.publish(p)
            self._call_set_pose(self._px, self._py, self._pz)
            self._publish_target_debug_marker()
            return
        t_elapsed = (now - self._t0).nanoseconds * 1e-9

        if self._last_tick_time is None:
            dt = 0.0
        else:
            dt = max(0.0, (now - self._last_tick_time).nanoseconds * 1e-9)
        self._last_tick_time = now

        vx, vy, vz = 0.0, 0.0, 0.0

        if self._phase == 'orbit':
            omega = 2.0 * math.pi * self._orbit_turns / self._orbit_duration_s
            if t_elapsed < self._orbit_duration_s:
                theta = self._theta0 + omega * t_elapsed
                self._px = self._orbit_r * math.cos(theta)
                self._py = self._orbit_r * math.sin(theta)
                if self._orbit_z_hold:
                    self._pz = self._orbit_z_start
                    vz = 0.0
                else:
                    self._pz += self._orbit_vz * dt
                    if self._pz < 0.0:
                        self._pz = 0.0
                    vz = float(self._orbit_vz)
                vx = -self._orbit_r * omega * math.sin(theta)
                vy = self._orbit_r * omega * math.cos(theta)
            else:
                self._phase = 'attack'
                theta_end = self._theta0 + 2.0 * math.pi * self._orbit_turns
                self._px = self._orbit_r * math.cos(theta_end)
                self._py = self._orbit_r * math.sin(theta_end)
                self._pz = self._orbit_z_start
                vx, vy, vz = self._velocity(t_elapsed)
                self._px += vx * dt
                self._py += vy * dt
                self._pz += vz * dt
                if self._pz < 0.0:
                    self._pz = 0.0
                self._snap_attack_pose_if_needed()
        else:
            vx, vy, vz = self._velocity(t_elapsed)
            self._px += vx * dt
            self._py += vy * dt
            self._pz += vz * dt
            if self._pz < 0.0:
                self._pz = 0.0
            self._snap_attack_pose_if_needed()

        tp = Point()
        tp.x, tp.y, tp.z = float(self._px), float(self._py), float(self._pz)
        self._trail.append(tp)

        if self._pub_point is not None:
            p = Point()
            p.x, p.y, p.z = float(self._px), float(self._py), float(self._pz)
            self._pub_point.publish(p)
        self._call_set_pose(self._px, self._py, self._pz)
        self._publish_target_debug_marker()

        if self._debug_log and (
            self._last_log_time is None
            or (now - self._last_log_time).nanoseconds * 1e-9 >= self._log_period
        ):
            self._last_log_time = now
            dist_xy = math.hypot(self._px, self._py)
            print(
                '[Target Debug]\n'
                f'phase={self._phase}\n'
                f'pos=({self._px:.3f},{self._py:.3f},{self._pz:.3f})\n'
                f'dist_to_center={dist_xy:.3f}\n'
                f'vel=({vx:.3f},{vy:.3f},{vz:.3f})',
                flush=True,
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetControllerNode()
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
