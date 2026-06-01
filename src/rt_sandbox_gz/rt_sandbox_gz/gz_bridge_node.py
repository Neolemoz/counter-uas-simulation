"""RT sandbox Gazebo bridge node — session-scoped entity lifecycle (PLAT-RT-G6)."""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from rt_sandbox_gz.gz_tools import (
    fmt_pose_req,
    fmt_remove_model_req,
    fmt_spawn_model_from_file,
    gz_service,
)
from rt_sandbox_gz.kinematic_plant import (
    AeroEnvironment,
    KinematicLimits,
    PlantState,
    integrate_toward_pose,
    is_mobile_entity,
    snap_to_commanded,
    telemetry_from_state,
)

ENTITY_MODEL_DIRS = {
    'drone': 'rt_drone',
    'radar': 'rt_radar',
    'interceptor': 'rt_interceptor',
    'waypoint_marker': 'rt_waypoint',
}

GROUND_SNAP_Z = {
    'drone': 0.5,
    'radar': 1.0,
    'interceptor': 0.3,
    'waypoint_marker': 0.4,
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


class RtSandboxGzBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('rt_sandbox_gz_bridge_node')
        self.declare_parameter('session_id', '')
        self.declare_parameter('world_name', 'rt_sandbox_flat')
        self.declare_parameter('ground_snap_enabled', True)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('kinematic_plant_enabled', True)
        self.declare_parameter('max_speed_mps', 25.0)
        self.declare_parameter('max_accel_mps2', 30.0)
        self.declare_parameter('max_turn_rate_rad_s', 2.5)
        self.declare_parameter('max_climb_mps', 8.0)
        self.declare_parameter('drag_decel_per_mps', 0.0)
        self.declare_parameter('wind_x_mps', 0.0)
        self.declare_parameter('wind_y_mps', 0.0)
        self.declare_parameter('wind_z_mps', 0.0)

        self._session_id = str(self.get_parameter('session_id').value).strip()
        self._ros_session_id = 's_' + self._session_id.replace('-', '_')
        self._world = str(self.get_parameter('world_name').value).strip()
        self._ground_snap = bool(self.get_parameter('ground_snap_enabled').value)
        self._rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._plant_enabled = bool(self.get_parameter('kinematic_plant_enabled').value)
        self._limits = KinematicLimits(
            max_speed_mps=max(0.1, float(self.get_parameter('max_speed_mps').value)),
            max_accel_mps2=max(0.1, float(self.get_parameter('max_accel_mps2').value)),
            max_turn_rate_rad_s=max(0.0, float(self.get_parameter('max_turn_rate_rad_s').value)),
            max_climb_mps=max(0.1, float(self.get_parameter('max_climb_mps').value)),
        )
        self._aero = AeroEnvironment(
            drag_decel_per_mps=max(0.0, float(self.get_parameter('drag_decel_per_mps').value)),
            wind_x_mps=float(self.get_parameter('wind_x_mps').value),
            wind_y_mps=float(self.get_parameter('wind_y_mps').value),
            wind_z_mps=float(self.get_parameter('wind_z_mps').value),
        )

        if not self._session_id:
            raise RuntimeError('session_id parameter required')

        prefix = f'/rt_sandbox/{self._ros_session_id}/'
        self._cmd_topic = f'{prefix}entity_pose_cmd'
        self._state_topic = f'{prefix}entity_state'

        self._entities: dict[str, dict[str, Any]] = {}
        self._sync_seq = 0
        self._model_paths = self._resolve_model_paths()
        self._timer_dt = 1.0 / self._rate_hz

        self.create_subscription(String, self._cmd_topic, self._on_cmd, 10)
        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        self.create_timer(self._timer_dt, self._on_timer)
        self.get_logger().info(
            f'RT sandbox GZ bridge listening on {self._cmd_topic} world={self._world} '
            f'kinematic_plant_enabled={self._plant_enabled}'
        )

    def _resolve_model_paths(self) -> dict[str, str]:
        share = get_package_share_directory('rt_sandbox_gz')
        out: dict[str, str] = {}
        for entity_type, model_dir in ENTITY_MODEL_DIRS.items():
            path = f'{share}/models/{model_dir}/model.sdf'
            out[entity_type] = path
        return out

    def _snap_z(self, entity_type: str, pose: dict[str, float]) -> dict[str, float]:
        out = dict(pose)
        if not self._ground_snap:
            return out
        offset = GROUND_SNAP_Z.get(entity_type, 0.0)
        out['z'] = float(out.get('z', 0.0)) + offset
        return out

    def _plant_state(self, ent: dict[str, Any]) -> PlantState:
        return PlantState.from_pose(ent.get('pose') or {}, ent.get('velocity'))

    def _advance_entity(
        self,
        ent: dict[str, Any],
        commanded: dict[str, float],
        dt: float,
        *,
        initial_spawn: bool = False,
    ) -> dict[str, float]:
        entity_type = str(ent.get('entity_type', 'drone'))
        ent['commanded_pose'] = dict(commanded)
        if (
            not self._plant_enabled
            or not is_mobile_entity(entity_type)
            or initial_spawn
        ):
            plant = snap_to_commanded(self._plant_state(ent), commanded)
        elif dt <= 0.0:
            plant = self._plant_state(ent)
        else:
            plant = integrate_toward_pose(
                self._plant_state(ent),
                commanded,
                dt,
                self._limits,
                self._aero,
            )

        ent['pose'] = plant.as_pose()
        ent['velocity'] = {'x': plant.vx, 'y': plant.vy, 'z': plant.vz}
        ent['last_integrate_monotonic'] = time.monotonic()
        return dict(ent['pose'])

    def _push_pose_to_gazebo(self, ent: dict[str, Any]) -> bool:
        sim_ref = str(ent.get('sim_entity_ref') or '')
        if not sim_ref:
            return False
        snapped = self._snap_z(str(ent.get('entity_type', 'drone')), dict(ent.get('pose') or {}))
        req = fmt_pose_req(
            sim_ref,
            float(snapped['x']),
            float(snapped['y']),
            float(snapped['z']),
            float(snapped.get('yaw_deg', 0.0)),
        )
        return gz_service(self._world, 'set_pose', req)

    def _entity_state_entry(self, ent: dict[str, Any]) -> dict[str, Any]:
        plant = self._plant_state(ent)
        telem = telemetry_from_state(plant)
        pose = dict(ent.get('pose') or {})
        return {
            'entity_id': ent['entity_id'],
            'entity_type': ent['entity_type'],
            'sim_entity_ref': ent.get('sim_entity_ref'),
            'pose': pose,
            'position': {
                'x': float(pose.get('x', 0.0)),
                'y': float(pose.get('y', 0.0)),
                'z': float(pose.get('z', 0.0)),
            },
            **telem,
            'target_state': 'none',
            'lifecycle_state': 'spawned',
        }

    def _on_cmd(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('invalid entity_pose_cmd JSON')
            return
        if data.get('schema') != 'rt_entity_pose_cmd_v1':
            return
        op = str(data.get('op', 'apply'))
        entity_id = str(data.get('entity_id', ''))
        if not entity_id:
            return
        if op == 'delete':
            self._delete_entity(entity_id, data)
            return
        self._apply_entity(entity_id, data)

    def _apply_entity(self, entity_id: str, data: dict[str, Any]) -> None:
        entity_type = str(data.get('entity_type', 'drone'))
        sim_ref = str(data.get('sim_entity_ref') or entity_id.replace('-', '')[:12])
        commanded = {
            'x': float((data.get('pose') or {}).get('x', 0.0)),
            'y': float((data.get('pose') or {}).get('y', 0.0)),
            'z': float((data.get('pose') or {}).get('z', 0.0)),
            'yaw_deg': float((data.get('pose') or {}).get('yaw_deg', 0.0)),
        }

        existing = self._entities.get(entity_id)
        if existing is None:
            ent = {
                'entity_id': entity_id,
                'entity_type': entity_type,
                'sim_entity_ref': sim_ref,
                'pose': dict(commanded),
                'commanded_pose': dict(commanded),
                'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'last_integrate_monotonic': time.monotonic(),
            }
            self._advance_entity(ent, commanded, 0.0, initial_spawn=True)
            sdf = self._model_paths.get(entity_type, self._model_paths['drone'])
            snapped = self._snap_z(entity_type, dict(ent['pose']))
            req = fmt_spawn_model_from_file(
                sdf,
                sim_ref,
                float(snapped['x']),
                float(snapped['y']),
                float(snapped['z']),
            )
            ok = gz_service(self._world, 'create', req)
            if not ok:
                self.get_logger().warning(f'spawn failed for {sim_ref}')
                return
            self._entities[entity_id] = ent
            self._sync_seq += 1
            self._publish_state()
            return

        now = time.monotonic()
        last = float(existing.get('last_integrate_monotonic') or now)
        dt = max(0.0, now - last)
        self._advance_entity(existing, commanded, dt)
        if not self._push_pose_to_gazebo(existing):
            self.get_logger().warning(f'set_pose failed for {sim_ref}')
            return
        self._sync_seq += 1
        self._publish_state()

    def _on_timer(self) -> None:
        if not self._plant_enabled or not self._entities:
            self._publish_state()
            return
        changed = False
        now = time.monotonic()
        for ent in self._entities.values():
            if not is_mobile_entity(str(ent.get('entity_type', 'drone'))):
                continue
            commanded = dict(ent.get('commanded_pose') or ent.get('pose') or {})
            last = float(ent.get('last_integrate_monotonic') or now)
            dt = max(0.0, now - last)
            if dt < 1e-6:
                continue
            before = dict(ent.get('pose') or {})
            self._advance_entity(ent, commanded, dt)
            after = dict(ent.get('pose') or {})
            if (
                abs(after.get('x', 0.0) - before.get('x', 0.0)) > 1e-6
                or abs(after.get('y', 0.0) - before.get('y', 0.0)) > 1e-6
                or abs(after.get('z', 0.0) - before.get('z', 0.0)) > 1e-6
            ):
                if self._push_pose_to_gazebo(ent):
                    changed = True
        if changed:
            self._sync_seq += 1
        self._publish_state()

    def _delete_entity(self, entity_id: str, data: dict[str, Any]) -> None:
        ent = self._entities.pop(entity_id, None)
        sim_ref = None
        if ent:
            sim_ref = ent.get('sim_entity_ref')
        if sim_ref is None:
            sim_ref = data.get('sim_entity_ref')
        if sim_ref:
            req = fmt_remove_model_req(str(sim_ref))
            gz_service(self._world, 'remove', req)
        self._sync_seq += 1
        self._publish_state()

    def _publish_state(self) -> None:
        payload = {
            'schema': 'rt_entity_state_v1',
            'timestamp_utc': _utc_now(),
            'sync_seq': self._sync_seq,
            'entities': [self._entity_state_entry(e) for e in self._entities.values()],
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._state_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RtSandboxGzBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
