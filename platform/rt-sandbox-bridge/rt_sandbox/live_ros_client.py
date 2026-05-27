"""Live-mode rclpy client for adapter worker subprocess (PLAT-RT-G6).

Bridge process must not import this module.
"""

from __future__ import annotations

import json
import threading
from datetime import datetime, timezone
from typing import Any

from rt_sandbox.entity_model_map import entity_state_to_feedback, snap_pose_for_gazebo
from rt_sandbox.ros_allowlist import session_topic_prefix


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


class LiveRosClient:
    """Session-scoped ROS publisher/subscriber for entity_pose_cmd / entity_state."""

    def __init__(
        self,
        session_id: str,
        *,
        ros_domain_id: int | None = None,
        ground_snap_enabled: bool = True,
    ) -> None:
        self._session_id = session_id
        self._ros_domain_id = ros_domain_id
        self._ground_snap_enabled = ground_snap_enabled
        self._prefix = session_topic_prefix(session_id)
        self._cmd_topic = f"{self._prefix}entity_pose_cmd"
        self._state_topic = f"{self._prefix}entity_state"
        self._node = None
        self._cmd_pub = None
        self._latest_state: dict[str, Any] | None = None
        self._lock = threading.Lock()
        self._started = False

    @property
    def available(self) -> bool:
        return self._started

    def start(self) -> bool:
        try:
            import os

            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
        except ImportError:
            return False

        if self._ros_domain_id is not None:
            os.environ["ROS_DOMAIN_ID"] = str(self._ros_domain_id)

        if not rclpy.ok():
            rclpy.init()

        node = Node("rt_adapter_worker_client")

        def _on_state(msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            if data.get("schema") != "rt_entity_state_v1":
                return
            with self._lock:
                self._latest_state = data

        node.create_subscription(String, self._state_topic, _on_state, 10)
        cmd_pub = node.create_publisher(String, self._cmd_topic, 10)

        self._node = node
        self._cmd_pub = cmd_pub
        self._String = String
        self._started = True

        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        return True

    def shutdown(self) -> None:
        if self._node is None:
            return
        try:
            import rclpy

            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        self._node = None
        self._started = False

    def publish_pose_cmd(
        self,
        *,
        op: str,
        entity_id: str,
        entity_type: str,
        pose: dict[str, float],
        sim_entity_ref: str | None = None,
        bridge_revision: int | None = None,
    ) -> None:
        if not self._started or self._cmd_pub is None:
            return
        cmd_pose = snap_pose_for_gazebo(
            entity_type,
            dict(pose),
            enabled=self._ground_snap_enabled,
        )
        payload: dict[str, Any] = {
            "schema": "rt_entity_pose_cmd_v1",
            "op": op,
            "entity_id": entity_id,
            "entity_type": entity_type,
            "pose": cmd_pose,
        }
        if sim_entity_ref:
            payload["sim_entity_ref"] = sim_entity_ref
        if bridge_revision is not None:
            payload["bridge_revision"] = bridge_revision
        msg = self._String()
        msg.data = json.dumps(payload)
        self._cmd_pub.publish(msg)

    def get_feedback(self) -> dict[str, Any] | None:
        with self._lock:
            state = dict(self._latest_state) if self._latest_state else None
        if state is None:
            return None
        return entity_state_to_feedback(
            state,
            ground_snap_enabled=self._ground_snap_enabled,
        )
