"""
RViz visualization for /tracks (geometry_msgs/Point only — no track_id in message).

We assign stable integer track IDs by matching each incoming point to the nearest
known track within TRACK_MATCH_M; otherwise start a new ID.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# geometry_msgs/Point has no ID — match updates to nearest track (meters)
TRACK_MATCH_M = 3.0
MAX_TRAIL_POINTS = 50

# Protected zone (map frame): flat red disk, radius 5 m
PROTECTED_ZONE_RADIUS_M = 5.0
PROTECTED_ZONE_HEIGHT_M = 0.1


def _priority(t: str) -> int:
    """LOW=0, MEDIUM=1, HIGH=2 — state machine + escalation alerts."""
    return {'LOW': 0, 'MEDIUM': 1, 'HIGH': 2}[t]


def _candidate_from_distance(distance: float, high_d: float, medium_d: float) -> str:
    """Raw threat from horizontal distance only. Never use for alerts, logs, or markers."""
    if distance < high_d:
        return 'HIGH'
    if distance < medium_d:
        return 'MEDIUM'
    return 'LOW'


def _state_machine_step(state: dict[str, Any], candidate: str) -> tuple[str | None, str, int]:
    """
    Production state machine: candidate (distance) → final threat in state.

    Returns (prev_threat, final_threat, frames_after) for DEBUG print.
    Mutates state['threat'] and state['frames'].
    """
    prev = state['threat']
    frames = int(state['frames'])

    # First observation: seed final from candidate once; candidate is not used after this for output
    if prev is None:
        state['threat'] = candidate
        state['frames'] = 0
        return (None, candidate, 0)

    # STEP 1: UPGRADE (immediate)
    if _priority(candidate) > _priority(prev):
        final = candidate
        frames = 0
    # STEP 2: SAME
    elif candidate == prev:
        final = prev
        frames += 1
    # STEP 3: DOWNGRADE (with HOLD) — single-step HIGH→MEDIUM→LOW, never jump from candidate
    else:
        if prev == 'HIGH':
            if frames >= 3:
                final = 'MEDIUM'
                frames = 0
            else:
                final = 'HIGH'
                frames += 1
        elif prev == 'MEDIUM':
            if frames >= 2:
                final = 'LOW'
                frames = 0
            else:
                final = 'MEDIUM'
                frames += 1
        else:
            final = 'LOW'

    state['threat'] = final
    state['frames'] = frames
    return (prev, final, frames)


@dataclass
class TrackVizState:
    track_id: int
    last: Point
    trail: list[Point] = field(default_factory=list)


def _dist(a: Point, b: Point) -> float:
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _copy_point(p: Point) -> Point:
    q = Point()
    q.x, q.y, q.z = p.x, p.y, p.z
    return q


class VizNode(Node):
    def __init__(self) -> None:
        super().__init__('viz_node')
        self.declare_parameter('scenario', 'single')
        self.declare_parameter('threat.medium_distance', 8.0)
        self.declare_parameter('threat.high_distance', 5.0)
        self._medium_d = float(self.get_parameter('threat.medium_distance').value)
        self._high_d = float(self.get_parameter('threat.high_distance').value)

        self._pub = self.create_publisher(MarkerArray, '/track_markers', 10)
        self.create_subscription(Point, '/tracks', self._on_track, 10)

        self._tracks: dict[int, TrackVizState] = {}
        self._next_id = 1
        # track_id → { "threat": str | None, "frames": int } — single source of truth for FINAL
        self._track_states: dict[int, dict[str, Any]] = {}
        # track_id → last threat level we *announced* (escalation only); updated only on print
        self._last_alerted: dict[int, str] = {}

        self.create_timer(0.1, self._publish_markers)

    def _ensure_track_state(self, tid: int) -> dict[str, Any]:
        if tid not in self._track_states:
            self._track_states[tid] = {'threat': None, 'frames': 0}
        return self._track_states[tid]

    def _on_track(self, msg: Point) -> None:
        incoming = _copy_point(msg)

        best_id: int | None = None
        best_d = float('inf')
        for tid, st in self._tracks.items():
            d = _dist(incoming, st.last)
            if d < best_d:
                best_d = d
                best_id = tid

        if best_id is not None and best_d <= TRACK_MATCH_M:
            tid = best_id
            st = self._tracks[tid]
            st.last = incoming
            st.trail.append(incoming)
            if len(st.trail) > MAX_TRAIL_POINTS:
                st.trail = st.trail[-MAX_TRAIL_POINTS:]
        else:
            tid = self._next_id
            self._next_id += 1
            self._tracks[tid] = TrackVizState(tid, incoming, trail=[incoming])

        st = self._tracks[tid]
        distance = math.sqrt(st.last.x ** 2 + st.last.y ** 2)
        self.get_logger().info(f'Track {tid} distance to zone: {distance:.2f}')

        # --- Candidate: distance only (never used for alerts / user logs / markers) ---
        candidate = _candidate_from_distance(distance, self._high_d, self._medium_d)

        state = self._ensure_track_state(tid)
        prev, final_threat, frames_after = _state_machine_step(state, candidate)

        print(
            f'[DEBUG] Track {tid} | prev={prev} cand={candidate} final={final_threat} '
            f'frames={frames_after}',
        )
        self.get_logger().info(f'Track {tid} Stable threat: {final_threat}')

        # --- Escalation-only alerts: print only when priority(final) > priority(previous final) ---
        # previous = prev from state machine (threat before this step). First sample: prev is None
        # (implicit LOW). De-escalations (HIGH→MEDIUM, MEDIUM→LOW, …) produce no console line.
        if prev is None:
            if final_threat == 'HIGH':
                print(f'🚨 ALERT: Track {tid} is HIGH threat!')
                self._last_alerted[tid] = final_threat
            elif final_threat == 'MEDIUM':
                print(f'⚠️ WARNING: Track {tid} is MEDIUM threat')
                self._last_alerted[tid] = final_threat
        elif _priority(final_threat) > _priority(prev):
            if final_threat == 'HIGH':
                print(f'🚨 ALERT: Track {tid} is HIGH threat!')
            elif final_threat == 'MEDIUM':
                print(f'⚠️ WARNING: Track {tid} is MEDIUM threat')
            self._last_alerted[tid] = final_threat

        self._publish_markers()

    def _final_threat_for_track(self, tid: int) -> str | None:
        return self._track_states.get(tid, {}).get('threat')

    def _publish_markers(self) -> None:
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        cyl = Marker()
        cyl.header.stamp = stamp
        cyl.header.frame_id = 'map'
        cyl.ns = 'protected_zone'
        cyl.id = 0
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose.position.x = 0.0
        cyl.pose.position.y = 0.0
        cyl.pose.position.z = 0.0
        cyl.pose.orientation.w = 1.0
        dia = 2.0 * PROTECTED_ZONE_RADIUS_M
        cyl.scale.x = dia
        cyl.scale.y = dia
        cyl.scale.z = PROTECTED_ZONE_HEIGHT_M
        cyl.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)
        ma.markers.append(cyl)

        for tid, st in self._tracks.items():
            base_id = tid * 10
            # --- Visualization: ONLY final state from track_states ---
            final_threat = self._final_threat_for_track(tid) or 'LOW'

            sph = Marker()
            sph.header.stamp = stamp
            sph.header.frame_id = 'map'
            sph.ns = 'track_viz'
            sph.id = base_id + 0
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position.x = st.last.x
            sph.pose.position.y = st.last.y
            sph.pose.position.z = st.last.z
            sph.pose.orientation.w = 1.0
            sph.scale.x = 0.5
            sph.scale.y = 0.5
            sph.scale.z = 0.5
            if final_threat == 'LOW':
                sph.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif final_threat == 'MEDIUM':
                sph.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            else:
                sph.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            ma.markers.append(sph)

            txt = Marker()
            txt.header.stamp = stamp
            txt.header.frame_id = 'map'
            txt.ns = 'track_viz'
            txt.id = base_id + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = st.last.x
            txt.pose.position.y = st.last.y
            txt.pose.position.z = st.last.z + 1.0
            txt.pose.orientation.w = 1.0
            txt.scale.x = 0.6
            txt.scale.y = 0.6
            txt.scale.z = 0.6
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f'ID: {tid}'
            ma.markers.append(txt)

            line = Marker()
            line.header.stamp = stamp
            line.header.frame_id = 'map'
            line.ns = 'track_viz'
            line.id = base_id + 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.1
            line.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            line.points = list(st.trail)
            ma.markers.append(line)

        self._pub.publish(ma)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
