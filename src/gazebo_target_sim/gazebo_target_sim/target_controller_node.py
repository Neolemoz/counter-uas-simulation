"""Drive Gazebo target pose via /world/<world>/set_pose (gz.msgs.Pose); sky approach then terminal dive."""

from __future__ import annotations

import os
import shutil
import subprocess
from typing import Sequence

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool

from gazebo_target_sim.clock_reset import subscribe_sim_time_reset
from gazebo_target_sim.gz_entity_tools import fmt_remove_model_req, fmt_spawn_model_from_file
from gazebo_target_sim.gz_pose_tools import fmt_pose_req


class TargetControllerNode(Node):
    """
    Calls ``gz service -s /world/<world>/set_pose`` at fixed rate (default 10 Hz).

    Counter-UAS hostile track: starts **high** (typ. above the defended point), then **terminal dive**
    (steeper ``vz``). Optional horizontal ``approach_speed`` (e.g. toward +x); for a pure **drop from
    sky** use ``approach_speed:=0``. Integrated as ``p += v * dt``.
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
        self.get_logger().info(
            f'Target pose -> {self._service} model={self._model!r} at {rate:.1f} Hz '
            f'(sky approach: vx={self._v_approach} vz={self._v_approach_z} m/s until t={self._t_dive}s, '
            f'then terminal dive vz={self._v_dive_z} m/s; start=({self._px},{self._py},{self._pz})){extra}',
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

    def _velocity(self, t_elapsed_s: float) -> tuple[float, float, float]:
        if t_elapsed_s < self._t_dive:
            return (self._v_approach, 0.0, self._v_approach_z)
        return (self._v_approach, 0.0, self._v_dive_z)

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
            return
        t_elapsed = (now - self._t0).nanoseconds * 1e-9

        if self._last_tick_time is None:
            dt = 0.0
        else:
            dt = max(0.0, (now - self._last_tick_time).nanoseconds * 1e-9)
        self._last_tick_time = now

        vx, vy, vz = self._velocity(t_elapsed)
        self._px += vx * dt
        self._py += vy * dt
        self._pz += vz * dt
        if self._pz < 0.0:
            self._pz = 0.0

        if self._pub_point is not None:
            p = Point()
            p.x, p.y, p.z = float(self._px), float(self._py), float(self._pz)
            self._pub_point.publish(p)
        self._call_set_pose(self._px, self._py, self._pz)

        if self._last_log_time is None or (now - self._last_log_time).nanoseconds * 1e-9 >= self._log_period:
            self._last_log_time = now
            phase = 'sky_approach' if t_elapsed < self._t_dive else 'terminal_dive'
            print(
                f'[target] phase={phase} pos=({self._px:.2f},{self._py:.2f},{self._pz:.2f})',
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
