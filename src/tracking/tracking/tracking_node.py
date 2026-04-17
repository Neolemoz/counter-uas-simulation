"""
Multi-target tracker: track–detection association, candidate confirmation (ghost
rejection), and track lifecycle (miss counter + deletion).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

# --- Track association — dual STRICT gating (meters) ---
# A pair (track, detection) is valid only if BOTH are true:
#   1) distance(det, track current position) < ASSOCIATION_GATE_M
#   2) distance(det, predicted next position) < ASSOCIATION_GATE_M
# Predicted position uses constant-velocity: vx = x - px, pred = x + vx (same for y,z).
ASSOCIATION_GATE_M = 2.0

# --- Candidate confirmation (same blob across frames) ---
# Detections within this distance (m) are treated as the same candidate.
CANDIDATE_MERGE_M = 1.0
# Number of consecutive matching frames required before creating a real track.
CONFIRMATION_HITS = 3

CYCLE_PERIOD_S = 0.1
MAX_MISSED_FRAMES = 10
# If two confirmed tracks are closer than this (meters), merge duplicates.
MERGE_DISTANCE_M = 1.0


@dataclass
class Track:
    """Confirmed target: pose, previous pose, lifecycle."""

    track_id: int
    x: float
    y: float
    z: float
    px: float
    py: float
    pz: float
    missed_frames: int = 0

    def predicted_xyz(self) -> tuple[float, float, float]:
        vx = self.x - self.px
        vy = self.y - self.py
        vz = self.z - self.pz
        return (self.x + vx, self.y + vy, self.z + vz)

    def update_from_detection(self, det: Point) -> None:
        self.px, self.py, self.pz = self.x, self.y, self.z
        self.x, self.y, self.z = det.x, det.y, det.z


@dataclass
class Candidate:
    """Temporary blob: not a track until confirmed (reduces ghost tracks)."""

    x: float
    y: float
    z: float
    hit_count: int


def _distance_point_to_xyz(det: Point, xyz: tuple[float, float, float]) -> float:
    dx = det.x - xyz[0]
    dy = det.y - xyz[1]
    dz = det.z - xyz[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _distance_point_to_candidate(det: Point, c: Candidate) -> float:
    dx = det.x - c.x
    dy = det.y - c.y
    dz = det.z - c.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _distance_det_to_track_current(det: Point, tr: Track) -> float:
    return _distance_point_to_xyz(det, (tr.x, tr.y, tr.z))


class TrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('tracking_node')
        self.declare_parameter('scenario', 'single')
        self._tracks: list[Track] = []
        self._candidates: list[Candidate] = []
        self._next_id = 1

        self._detection_buffer: list[Point] = []

        self._pub = self.create_publisher(Point, '/tracks', 10)
        # Input changed: use fused detections (radar + camera)
        self.create_subscription(Point, '/fused_detections', self._on_detection, 10)
        self._timer = self.create_timer(CYCLE_PERIOD_S, self._on_cycle_timer)
        self.get_logger().info('Tracking using fused detections')

    def _on_detection(self, msg: Point) -> None:
        p = Point()
        p.x, p.y, p.z = msg.x, msg.y, msg.z
        self._detection_buffer.append(p)

    def _on_cycle_timer(self) -> None:
        detections = self._detection_buffer
        self._detection_buffer = []
        self._process_cycle(detections)

    def _process_cycle(self, detections: list[Point]) -> None:
        n_old = len(self._tracks)

        # ==================================================================
        # TRACK ASSOCIATION — greedy match with dual gating
        # Valid match only if BOTH:
        #   d_curr = dist(det, track current) < 2.0
        #   d_pred = dist(det, predicted next) < 2.0   where pred = current + (current - prev)
        # ==================================================================
        assoc_pairs: list[tuple[float, float, int, int]] = []
        for j, det in enumerate(detections):
            for i, tr in enumerate(self._tracks):
                d_curr = _distance_det_to_track_current(det, tr)
                pred = tr.predicted_xyz()
                d_pred = _distance_point_to_xyz(det, pred)
                if d_curr < ASSOCIATION_GATE_M and d_pred < ASSOCIATION_GATE_M:
                    # Sort primarily by predicted consistency, then current proximity
                    assoc_pairs.append((d_pred, d_curr, i, j))

        assoc_pairs.sort(key=lambda x: (x[0], x[1]))

        matched_track_idx: set[int] = set()
        matched_det_idx: set[int] = set()

        for _d_pred, _d_curr, ti, dj in assoc_pairs:
            if ti in matched_track_idx or dj in matched_det_idx:
                continue
            matched_track_idx.add(ti)
            matched_det_idx.add(dj)
            self._tracks[ti].update_from_detection(detections[dj])
            self._tracks[ti].missed_frames = 0
            tid = self._tracks[ti].track_id
            self.get_logger().info(f'Track {tid} updated (valid match)')

        # Tracks that were not matched: log "bad match" if some detection was
        # close to current OR predicted but not both (prevents silent bad updates).
        for i in range(n_old):
            if i in matched_track_idx:
                continue
            tr = self._tracks[i]
            pred = tr.predicted_xyz()
            for det in detections:
                d_curr = _distance_det_to_track_current(det, tr)
                d_pred = _distance_point_to_xyz(det, pred)
                ok = d_curr < ASSOCIATION_GATE_M and d_pred < ASSOCIATION_GATE_M
                partial = (d_curr < ASSOCIATION_GATE_M or d_pred < ASSOCIATION_GATE_M) and not ok
                if partial:
                    self.get_logger().info(
                        f'Track {tr.track_id} rejected update (bad match)',
                    )
                    break

        # ==================================================================
        # TRACK LIFECYCLE — miss counter
        # ==================================================================
        for i in range(n_old):
            if i not in matched_track_idx:
                self._tracks[i].missed_frames += 1
                tid = self._tracks[i].track_id
                mf = self._tracks[i].missed_frames
                self.get_logger().info(
                    f'Track {tid} NOT updated (missed_frames={mf})',
                )

        # ==================================================================
        # CANDIDATES — only unmatched detections can form / reinforce candidates
        # ==================================================================
        unmatched_dets: list[Point] = [
            detections[j] for j in range(len(detections)) if j not in matched_det_idx
        ]
        self._process_candidates(unmatched_dets)

        # ==================================================================
        # TRACK LIFECYCLE — delete stale tracks
        # ==================================================================
        removed = [tr for tr in self._tracks if tr.missed_frames > MAX_MISSED_FRAMES]
        for tr in removed:
            self.get_logger().info(
                f'Track deleted: track_id={tr.track_id} '
                f'(missed_frames={tr.missed_frames} > {MAX_MISSED_FRAMES})',
            )
        self._tracks = [tr for tr in self._tracks if tr.missed_frames <= MAX_MISSED_FRAMES]

        # ==================================================================
        # MERGE DUPLICATES — after updates, merge tracks that are too close
        # Keep the older (smaller) track_id and remove the newer one.
        # ==================================================================
        self._merge_duplicate_tracks()

        self._publish_all_tracks()
        self._log_active_tracks()
        self._log_all_tracks()

    def _merge_duplicate_tracks(self) -> None:
        """
        If two tracks are within MERGE_DISTANCE_M, treat them as duplicates:
        keep the older track_id (smaller) and remove the newer one.
        """
        i = 0
        while i < len(self._tracks):
            j = i + 1
            while j < len(self._tracks):
                a = self._tracks[i]
                b = self._tracks[j]
                d = math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
                if d < MERGE_DISTANCE_M:
                    older, newer = (a, b) if a.track_id < b.track_id else (b, a)
                    self.get_logger().info(
                        f'Merging track {older.track_id} and track {newer.track_id}',
                    )
                    # Remove the newer track
                    if newer is a:
                        self._tracks.pop(i)
                        i = max(0, i - 1)
                        break
                    self._tracks.pop(j)
                    continue
                j += 1
            else:
                i += 1

    def _process_candidates(self, unmatched_dets: list[Point]) -> None:
        """
        Candidate logic (separate from tracks):

        * Match unmatched detections to existing candidates within CANDIDATE_MERGE_M.
        * Increment hit_count on match; at CONFIRMATION_HITS → real track.
        * Candidates with no matching detection this frame are discarded.
        * Unmatched detections spawn new candidates (hit_count = 1).
        """
        if not unmatched_dets and not self._candidates:
            return

        cand_pairs: list[tuple[float, int, int]] = []
        for j, det in enumerate(unmatched_dets):
            for k, cand in enumerate(self._candidates):
                d = _distance_point_to_candidate(det, cand)
                if d < CANDIDATE_MERGE_M:
                    cand_pairs.append((d, k, j))

        cand_pairs.sort(key=lambda x: x[0])

        matched_cand_idx: set[int] = set()
        matched_unmatched_det_idx: set[int] = set()

        confirmed_indices: set[int] = set()

        for _d, k, j in cand_pairs:
            if k in matched_cand_idx or j in matched_unmatched_det_idx:
                continue
            matched_cand_idx.add(k)
            matched_unmatched_det_idx.add(j)

            cand = self._candidates[k]
            det = unmatched_dets[j]
            cand.x, cand.y, cand.z = det.x, det.y, det.z
            cand.hit_count += 1

            if cand.hit_count >= CONFIRMATION_HITS:
                tid = self._next_id
                self._next_id += 1
                self._tracks.append(
                    Track(tid, cand.x, cand.y, cand.z, cand.x, cand.y, cand.z, missed_frames=0),
                )
                confirmed_indices.add(k)
                self.get_logger().info(
                    f'Candidate confirmed → Track created: track_id={tid} at '
                    f'x={cand.x:.3f}, y={cand.y:.3f}, z={cand.z:.3f}',
                )

        # Keep matched candidates that were not just promoted to tracks.
        new_candidates: list[Candidate] = []
        for k, cand in enumerate(self._candidates):
            if k in confirmed_indices:
                continue
            if k in matched_cand_idx:
                new_candidates.append(cand)
            else:
                self.get_logger().info(
                    f'Candidate discarded: position=({cand.x:.3f}, {cand.y:.3f}, {cand.z:.3f}) '
                    f'hit_count was {cand.hit_count}',
                )

        self._candidates = new_candidates

        # Spawn new candidates for detections that did not match any candidate.
        for j, det in enumerate(unmatched_dets):
            if j in matched_unmatched_det_idx:
                continue
            self._candidates.append(Candidate(det.x, det.y, det.z, 1))
            self.get_logger().info(
                f'Candidate detected: position=({det.x:.3f}, {det.y:.3f}, {det.z:.3f}) '
                f'hit_count=1',
            )

    def _publish_all_tracks(self) -> None:
        for tr in self._tracks:
            out = Point()
            out.x = tr.x
            out.y = tr.y
            out.z = tr.z
            self._pub.publish(out)

    def _log_active_tracks(self) -> None:
        ids = [tr.track_id for tr in self._tracks]
        self.get_logger().info(
            f'Active tracks: {len(self._tracks)} — track_ids: {ids}',
        )

    def _log_all_tracks(self) -> None:
        for tr in self._tracks:
            self.get_logger().info(
                f'Track {tr.track_id}: x={tr.x:.3f}, y={tr.y:.3f}, z={tr.z:.3f} '
                f'(missed_frames={tr.missed_frames})',
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
