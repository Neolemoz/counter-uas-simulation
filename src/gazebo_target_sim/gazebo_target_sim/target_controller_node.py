"""Drive Gazebo target pose via /world/<world>/set_pose (gz.msgs.Pose); mirrors world_sim-style orbit + Z sine."""

from __future__ import annotations

import math
import shutil
import subprocess
from typing import Sequence

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool

from gazebo_target_sim.gz_pose_tools import fmt_pose_req


class TargetControllerNode(Node):
    """
    Calls ``gz service -s /world/<world>/set_pose`` at fixed rate (default 10 Hz).
    Trajectory: horizontal circle + sinusoidal altitude (same spirit as ``world_sim_node``).
    """

    def __init__(self) -> None:
        super().__init__('target_controller_node')
        self.declare_parameter('world_name', 'counter_uas_target')
        self.declare_parameter('model_name', 'sphere_target')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('radius_m', 10.0)
        self.declare_parameter('orbit_speed', 3.0)
        self.declare_parameter('z_base_m', 5.0)
        self.declare_parameter('z_amplitude_m', 0.5)
        self.declare_parameter('z_omega_rad_s', 0.55)
        self.declare_parameter('service_timeout_ms', 800)
        self.declare_parameter('publish_drone_position', True)
        self.declare_parameter('drone_position_topic', '/drone/position')
        self.declare_parameter('stop_topic', '/target/stop')

        self._world = str(self.get_parameter('world_name').value).strip()
        self._model = str(self.get_parameter('model_name').value).strip()
        self._cx = float(self.get_parameter('center_x').value)
        self._cy = float(self.get_parameter('center_y').value)
        self._r = float(self.get_parameter('radius_m').value)
        self._orbit = float(self.get_parameter('orbit_speed').value)
        self._z0 = float(self.get_parameter('z_base_m').value)
        self._z_a = float(self.get_parameter('z_amplitude_m').value)
        self._z_w = float(self.get_parameter('z_omega_rad_s').value)
        timeout_ms = int(self.get_parameter('service_timeout_ms').value)
        self._timeout_s = max(float(timeout_ms) * 1e-3, 0.05)
        self._pub_gt = bool(self.get_parameter('publish_drone_position').value)
        gt_topic = str(self.get_parameter('drone_position_topic').value).strip()
        self._stop_topic = str(self.get_parameter('stop_topic').value).strip()
        self._stopped = False

        rate = max(float(self.get_parameter('rate_hz').value), 0.5)
        self._gz = shutil.which('gz')
        if not self._gz:
            self.get_logger().fatal('gz CLI not found in PATH; install Gazebo / gz-sim.')
            raise RuntimeError('gz not found')

        self._service = f'/world/{self._world}/set_pose'
        self._t0 = self.get_clock().now()
        self._warned_fail = False
        self._pub_point = self.create_publisher(Point, gt_topic, 10) if self._pub_gt else None
        self.create_subscription(Bool, self._stop_topic, self._on_stop, 10)
        period = 1.0 / rate
        self._timer = self.create_timer(period, self._on_timer)
        extra = f' + {gt_topic} (ground truth)' if self._pub_gt else ''
        self.get_logger().info(
            f'Target pose -> {self._service} model={self._model!r} at {rate:.1f} Hz '
            f'(circle R={self._r} m, z_sine A={self._z_a} m){extra}',
        )

    def _on_stop(self, msg: Bool) -> None:
        if self._stopped:
            return
        if bool(msg.data):
            self._stopped = True
            self.get_logger().info('Target controller STOP received; freezing pose updates (gravity will act).')

    def _sample_xyz(self, t_s: float) -> tuple[float, float, float]:
        tau = self._orbit * t_s
        x = self._cx + self._r * math.cos(tau)
        y = self._cy + self._r * math.sin(tau)
        z = self._z0 + self._z_a * math.sin(self._z_w * t_s)
        return (x, y, z)

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
        if self._stopped:
            return
        t_s = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        x, y, z = self._sample_xyz(t_s)
        if self._pub_point is not None:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), float(z)
            self._pub_point.publish(p)
        self._call_set_pose(x, y, z)


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
