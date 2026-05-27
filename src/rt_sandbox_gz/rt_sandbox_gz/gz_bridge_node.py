"""RT sandbox Gazebo bridge node — session-scoped entity lifecycle (PLAT-RT-G6)."""

from __future__ import annotations

import json
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

        self._session_id = str(self.get_parameter('session_id').value).strip()
        self._world = str(self.get_parameter('world_name').value).strip()
        self._ground_snap = bool(self.get_parameter('ground_snap_enabled').value)
        self._rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        if not self._session_id:
            raise RuntimeError('session_id parameter required')

        prefix = f'/rt_sandbox/{self._session_id}/'
        self._cmd_topic = f'{prefix}entity_pose_cmd'
        self._state_topic = f'{prefix}entity_state'

        self._entities: dict[str, dict[str, Any]] = {}
        self._sync_seq = 0
        self._model_paths = self._resolve_model_paths()

        self.create_subscription(String, self._cmd_topic, self._on_cmd, 10)
        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        self.create_timer(1.0 / self._rate_hz, self._publish_state)
        self.get_logger().info(
            f'RT sandbox GZ bridge listening on {self._cmd_topic} world={self._world}'
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
        pose = self._snap_z(entity_type, dict(data.get('pose') or {}))
        x = float(pose.get('x', 0.0))
        y = float(pose.get('y', 0.0))
        z = float(pose.get('z', 0.0))
        yaw = float(pose.get('yaw_deg', 0.0))

        existing = self._entities.get(entity_id)
        if existing is None:
            sdf = self._model_paths.get(entity_type, self._model_paths['drone'])
            req = fmt_spawn_model_from_file(sdf, sim_ref, x, y, z)
            ok = gz_service(self._world, 'create', req)
            if not ok:
                self.get_logger().warning(f'spawn failed for {sim_ref}')
                return
            self._entities[entity_id] = {
                'entity_id': entity_id,
                'entity_type': entity_type,
                'sim_entity_ref': sim_ref,
                'pose': {'x': x, 'y': y, 'z': z, 'yaw_deg': yaw},
            }
            self._sync_seq += 1
            self._publish_state()
            return

        sim_ref = str(existing.get('sim_entity_ref') or sim_ref)
        req = fmt_pose_req(sim_ref, x, y, z, yaw)
        ok = gz_service(self._world, 'set_pose', req)
        if not ok:
            self.get_logger().warning(f'set_pose failed for {sim_ref}')
            return
        existing['pose'] = {'x': x, 'y': y, 'z': z, 'yaw_deg': yaw}
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
            'entities': [
                {
                    'entity_id': e['entity_id'],
                    'entity_type': e['entity_type'],
                    'sim_entity_ref': e.get('sim_entity_ref'),
                    'pose': dict(e.get('pose') or {}),
                }
                for e in self._entities.values()
            ],
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
