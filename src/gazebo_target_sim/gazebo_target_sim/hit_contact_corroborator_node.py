"""Subscribe to soft ImpactEvent and optional gz-sim Contacts (bridged) — log confirm / soft-only."""

from __future__ import annotations

from collections import deque
from typing import Deque, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts

from gazebo_target_sim_interfaces.msg import ImpactEvent


def _entity_names_contain(hay: str, needle: str) -> bool:
    if not needle or not hay:
        return False
    return needle in hay


def _contacts_relevant(c: Contacts, interceptor_id: str, target_substr: str) -> bool:
    for ct in c.contacts:
        n1 = ct.collision1.name
        n2 = ct.collision2.name
        pair = f'{n1} {n2}'
        if target_substr and interceptor_id:
            if (_entity_names_contain(n1, interceptor_id) or _entity_names_contain(n2, interceptor_id)) and (
                _entity_names_contain(n1, target_substr) or _entity_names_contain(n2, target_substr)
            ):
                return True
        elif interceptor_id:
            if _entity_names_contain(pair, interceptor_id):
                return True
    return False


class HitContactCorroboratorNode(Node):
    """
    When ``contacts_topic`` is set, buffers recent ``ros_gz_interfaces/Contacts`` and on each
    ``ImpactEvent`` checks for a matching pair within ``match_window_s``.
    """

    def __init__(self) -> None:
        super().__init__('hit_contact_corroborator')
        self.declare_parameter('impact_event_topic', '/interception/impact_event')
        self.declare_parameter('contacts_topic', '')
        self.declare_parameter('match_window_s', 0.2)
        self.declare_parameter(
            'target_label_models',
            'target_0=sphere_target_0,target_1=sphere_target_1,target_2=sphere_target_2',
        )
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        imp = str(self.get_parameter('impact_event_topic').value).strip()
        self._contacts_topic = str(self.get_parameter('contacts_topic').value).strip()
        self._match_w = max(0.02, float(self.get_parameter('match_window_s').value))
        raw = str(self.get_parameter('target_label_models').value).strip()
        self._label_map: dict[str, str] = {}
        if raw:
            for part in raw.split(','):
                part = part.strip()
                if '=' in part:
                    k, v = part.split('=', 1)
                    self._label_map[k.strip()] = v.strip()
        self._buf: Deque[Tuple[int, Contacts]] = deque(maxlen=80)
        self.create_subscription(ImpactEvent, imp, self._on_impact, 10)
        if self._contacts_topic:
            self.create_subscription(Contacts, self._contacts_topic, self._on_contacts, qos)
            self.get_logger().info(
                f'Corroborator: impact={imp!r} + contacts={self._contacts_topic!r} (window {self._match_w}s)',
            )
        else:
            self.get_logger().info(
                f'Corroborator: impact={imp!r} only (no contacts_topic — log soft hits as [IMPACT_SOFT])',
            )

    def _target_substr_for(self, label: str) -> str:
        if label in self._label_map:
            return self._label_map[label]
        if label:
            return label
        return 'sphere_target'

    def _on_contacts(self, msg: Contacts) -> None:
        now = self.get_clock().now().nanoseconds
        self._buf.append((now, msg))

    def _on_impact(self, msg: ImpactEvent) -> None:
        iid = str(msg.interceptor_id).strip()
        tlab = str(msg.target_label).strip()
        if not self._contacts_topic:
            self.get_logger().info(f'[IMPACT_SOFT] interceptor={iid!r} target_label={tlab!r} (no Gazebo contacts bridge)')
            return
        tsub = self._target_substr_for(tlab) if tlab else 'sphere_target'
        t0 = self.get_clock().now().nanoseconds - int(self._match_w * 1e9)
        hit = False
        for ts, c in reversed(self._buf):
            if ts < t0:
                break
            if _contacts_relevant(c, iid, tsub):
                hit = True
                break
        if hit:
            self.get_logger().info(
                f'[HIT_CORROBORATED] soft impact + Gazebo contact within {self._match_w}s '
                f'(interceptor={iid!r} target_match={tsub!r})',
            )
        else:
            self.get_logger().warning(
                f'[IMPACT_SOFT_ONLY] no matching Gazebo contact in window for '
                f'interceptor={iid!r} target_match={tsub!r}',
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HitContactCorroboratorNode()
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
