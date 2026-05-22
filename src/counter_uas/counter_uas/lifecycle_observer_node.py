from __future__ import annotations

import math
import re

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String


_TRACK_ID_RE = re.compile(r"^track_(?P<track_id>.+)$")


def _obs_value(value: object) -> str:
    if value is None:
        return '(none)'
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        if math.isfinite(value):
            return f'{value:.3f}'
        return 'nan'
    text = str(value).strip()
    return text if text else '(empty)'


def _obs_line(tag: str, **fields: object) -> str:
    parts = [tag]
    for key, value in fields.items():
        parts.append(f'{key}={_obs_value(value)}')
    return ' '.join(parts)


def _extract_track_id(frame_id: str) -> str | None:
    match = _TRACK_ID_RE.match(frame_id.strip()) if frame_id else None
    if match:
        return match.group('track_id').strip() or None
    return frame_id.strip() or None


class LifecycleObserverNode(Node):
    """Passive bringup observer for track continuity and selection evidence."""

    def __init__(self) -> None:
        super().__init__('lifecycle_observer_node')

        self.declare_parameter('tracks_state_topic', '/tracks/state')
        self.declare_parameter('selected_id_topic', '/interceptor/selected_id')
        self.declare_parameter('summary_period_s', 5.0)
        self.declare_parameter('gap_warn_s', 0.30)
        self.declare_parameter('persistence_warn_s', 1.00)

        self._tracks_state_topic = str(self.get_parameter('tracks_state_topic').value)
        self._selected_id_topic = str(self.get_parameter('selected_id_topic').value)
        self._summary_period_s = float(self.get_parameter('summary_period_s').value)
        self._gap_warn_s = float(self.get_parameter('gap_warn_s').value)
        self._persistence_warn_s = float(self.get_parameter('persistence_warn_s').value)

        self._tracks_state_msgs_total = 0
        self._tracks_state_msgs_window = 0
        self._track_ids_window: set[str] = set()
        self._track_id_changes_window = 0
        self._track_gap_events_window = 0
        self._track_persistence_events_window = 0
        self._selected_id_changes_window = 0

        self._last_track_id: str | None = None
        self._last_track_stamp: Time | None = None
        self._current_track_start_stamp: Time | None = None
        self._last_selected_id: str | None = None

        self._tracks_sub = self.create_subscription(Odometry, self._tracks_state_topic, self._on_tracks_state, 10)
        self._selected_sub = self.create_subscription(String, self._selected_id_topic, self._on_selected_id, 10)
        self._summary_timer = self.create_timer(self._summary_period_s, self._on_summary)

        self.get_logger().info(
            _obs_line(
                '[LIFECYCLE_OBSERVER]',
                event='armed',
                tracks_state_topic=self._tracks_state_topic,
                selected_id_topic=self._selected_id_topic,
                summary_period_s=self._summary_period_s,
                gap_warn_s=self._gap_warn_s,
                persistence_warn_s=self._persistence_warn_s,
            ),
        )

    def _on_tracks_state(self, msg: Odometry) -> None:
        self._tracks_state_msgs_total += 1
        self._tracks_state_msgs_window += 1

        track_id = _extract_track_id(msg.child_frame_id)
        if track_id is not None:
            self._track_ids_window.add(track_id)

        current_stamp = Time.from_msg(msg.header.stamp)
        if self._last_track_stamp is not None:
            gap_s = (current_stamp - self._last_track_stamp).nanoseconds / 1e9
            if gap_s >= self._gap_warn_s:
                self._track_gap_events_window += 1
                self.get_logger().info(
                    _obs_line(
                        '[TRACK_CONTINUITY]',
                        event='track_gap',
                        gap_s=gap_s,
                        prev_track_id=self._last_track_id,
                        next_track_id=track_id,
                        frame_id=msg.child_frame_id,
                    ),
                )

        if track_id != self._last_track_id:
            if self._last_track_id is None:
                self._current_track_start_stamp = current_stamp
                self.get_logger().info(
                    _obs_line(
                        '[TRACK_CONTINUITY]',
                        event='track_first_seen',
                        track_id=track_id,
                        frame_id=msg.child_frame_id,
                    ),
                )
            else:
                self._track_id_changes_window += 1
                gap_s = None
                if self._last_track_stamp is not None:
                    gap_s = (current_stamp - self._last_track_stamp).nanoseconds / 1e9
                self.get_logger().info(
                    _obs_line(
                        '[TRACK_CONTINUITY]',
                        event='track_id_change',
                        prev_track_id=self._last_track_id,
                        next_track_id=track_id,
                        gap_s=gap_s,
                        frame_id=msg.child_frame_id,
                    ),
                )
                if self._current_track_start_stamp is not None and self._last_track_id is not None:
                    persistence_s = (current_stamp - self._current_track_start_stamp).nanoseconds / 1e9
                    if persistence_s >= self._persistence_warn_s:
                        self._track_persistence_events_window += 1
                        self.get_logger().info(
                            _obs_line(
                                '[SELECTION_PROXY]',
                                event='track_persistence_window',
                                track_id=self._last_track_id,
                                persistence_s=persistence_s,
                                track_switch_to=track_id,
                                frame_id=msg.child_frame_id,
                            ),
                        )
                        self.get_logger().info(
                            _obs_line(
                                '[TRACK_PERSISTENCE]',
                                event='track_persistence_boundary',
                                previous_track_id=self._last_track_id,
                                next_track_id=track_id,
                                persistence_s=persistence_s,
                            ),
                        )
                self._current_track_start_stamp = current_stamp

        self._last_track_id = track_id
        self._last_track_stamp = current_stamp

    def _on_selected_id(self, msg: String) -> None:
        selected_id = msg.data.strip() or None
        if selected_id == self._last_selected_id:
            return
        self._selected_id_changes_window += 1
        self.get_logger().info(
            _obs_line(
                '[LIFECYCLE_OBSERVER]',
                event='selected_id',
                selected_id=selected_id,
                previous_selected_id=self._last_selected_id,
            ),
        )
        self._last_selected_id = selected_id

    def _on_summary(self) -> None:
        now = self.get_clock().now()
        idle_s = None
        if self._last_track_stamp is not None:
            idle_s = (now - self._last_track_stamp).nanoseconds / 1e9
        self.get_logger().info(
            _obs_line(
                '[LIFECYCLE_OBSERVER]',
                event='summary',
                tracks_state_msgs_window=self._tracks_state_msgs_window,
                tracks_state_msgs_total=self._tracks_state_msgs_total,
                unique_track_ids_window=len(self._track_ids_window),
                track_id_changes_window=self._track_id_changes_window,
                track_gap_events_window=self._track_gap_events_window,
                track_persistence_events_window=self._track_persistence_events_window,
                selected_id_changes_window=self._selected_id_changes_window,
                idle_s=idle_s,
                last_track_id=self._last_track_id,
                selected_id=self._last_selected_id,
            ),
        )
        self._tracks_state_msgs_window = 0
        self._track_ids_window.clear()
        self._track_id_changes_window = 0
        self._track_gap_events_window = 0
        self._track_persistence_events_window = 0
        self._selected_id_changes_window = 0


def main() -> int:
    rclpy.init()
    node = LifecycleObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
