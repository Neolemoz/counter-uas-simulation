"""
Multi-target tracker: track–detection association, candidate confirmation (ghost
rejection), track lifecycle (miss counter + deletion), and CV Kalman filtering
on each confirmed track (state [x,y,z,vx,vy,vz], covariance P).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node

# --- Track association — dual gating (meters), now ROS-tunable ---
# A pair (track, detection) is valid only if BOTH are true:
#   1) distance(det, track predicted position this cycle) < association_gate_m
#   2) distance(det, one-step-ahead extrapolation) < association_gate_m
# Positions come from the KF **after** the time-update (predict) for this cycle, so for a
# well-tracked CV target this gate only needs to swallow measurement noise, not target speed.
ASSOCIATION_GATE_M = 2.0

# --- Candidate confirmation (same blob across frames) ---
# Detections within this distance (m) of the candidate's *predicted* position (NOT raw last
# position — important when target speed * dt > gate, e.g. 38 m/s @ 10 Hz = 3.8 m / frame
# vs gate 1.0 m which previously prevented any track from ever forming for fast targets).
CANDIDATE_MERGE_M = 1.0
# Number of consecutive matching frames required before creating a real track.
CONFIRMATION_HITS = 3

CYCLE_PERIOD_S = 0.1
# Last N positions per candidate for averaged finite-difference v_init (N = maxlen).
CANDIDATE_POS_HISTORY_MAX = 4
MAX_MISSED_FRAMES = 10
# If two confirmed tracks are closer than this (meters), merge duplicates.
MERGE_DISTANCE_M = 1.0

# --- Constant-velocity Kalman (fixed Q, R; dt = CYCLE_PERIOD_S) ---
# Process noise (tuned for ~10 Hz); grows uncertainty during coast.
_Q_POS = 0.02
_Q_VEL = 0.08
# Measurement noise (m^2) ~ 0.5 m std on fused position
_R_MEAS_VAR = 0.5**2
# Extra covariance inflation when no measurement is associated this cycle
_Q_MISS_POS = 0.15
_Q_MISS_VEL = 0.02
# Initial covariance for new tracks (candidate confirmed)
_P_INIT_POS_VAR = 4.0
_P_INIT_VEL_VAR = 25.0


def _symmetrize_psd(P: np.ndarray) -> None:
    P[:] = 0.5 * (P + P.T)


def _build_F(dt: float) -> np.ndarray:
    F = np.eye(6, dtype=float)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    return F


def _build_H() -> np.ndarray:
    H = np.zeros((3, 6), dtype=float)
    H[0, 0] = H[1, 1] = H[2, 2] = 1.0
    return H


def _build_Q(dt: float) -> np.ndarray:
    """Simple diagonal Q scaled by dt (constant-velocity random walk)."""
    q = np.zeros((6, 6), dtype=float)
    q[0, 0] = q[1, 1] = q[2, 2] = _Q_POS * dt
    q[3, 3] = q[4, 4] = q[5, 5] = _Q_VEL * dt
    return q


def _build_R() -> np.ndarray:
    return np.eye(3, dtype=float) * _R_MEAS_VAR


def _build_Q_miss() -> np.ndarray:
    q = np.zeros((6, 6), dtype=float)
    q[0, 0] = q[1, 1] = q[2, 2] = _Q_MISS_POS
    q[3, 3] = q[4, 4] = q[5, 5] = _Q_MISS_VEL
    return q


def _initial_P() -> np.ndarray:
    d = np.array(
        [
            _P_INIT_POS_VAR,
            _P_INIT_POS_VAR,
            _P_INIT_POS_VAR,
            _P_INIT_VEL_VAR,
            _P_INIT_VEL_VAR,
            _P_INIT_VEL_VAR,
        ],
        dtype=float,
    )
    return np.diag(d)


@dataclass
class Track:
    """Confirmed target: CV Kalman state, covariance, lifecycle."""

    track_id: int
    state: np.ndarray = field(repr=False)  # shape (6,) [x,y,z,vx,vy,vz]
    P: np.ndarray = field(repr=False)  # shape (6, 6)
    missed_frames: int = 0

    @property
    def x(self) -> float:
        return float(self.state[0])

    @property
    def y(self) -> float:
        return float(self.state[1])

    @property
    def z(self) -> float:
        return float(self.state[2])

    def position_xyz(self) -> tuple[float, float, float]:
        """Position after the latest predict (or update) for this cycle."""
        return (float(self.state[0]), float(self.state[1]), float(self.state[2]))

    def gate_ahead_xyz(self, dt: float) -> tuple[float, float, float]:
        """One-step linear extrapolation from current state (dual-gate second test)."""
        return (
            float(self.state[0] + self.state[3] * dt),
            float(self.state[1] + self.state[4] * dt),
            float(self.state[2] + self.state[5] * dt),
        )

    @staticmethod
    def new_from_position(
        tid: int,
        px: float,
        py: float,
        pz: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
    ) -> Track:
        s = np.array([px, py, pz, vx, vy, vz], dtype=float)
        return Track(track_id=tid, state=s, P=_initial_P(), missed_frames=0)

    def kf_predict(self, F: np.ndarray, Q: np.ndarray) -> None:
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + Q
        _symmetrize_psd(self.P)

    def kf_update(self, det: Point, H: np.ndarray, R: np.ndarray) -> None:
        z = np.array([det.x, det.y, det.z], dtype=float)
        y = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        ih = np.eye(6, dtype=float) - K @ H
        self.P = ih @ self.P
        _symmetrize_psd(self.P)

    def kf_miss_inflate(self, Q_miss: np.ndarray) -> None:
        self.P = self.P + Q_miss
        _symmetrize_psd(self.P)


def _new_candidate_pos_history() -> deque[tuple[float, float, float, float]]:
    return deque(maxlen=CANDIDATE_POS_HISTORY_MAX)


@dataclass
class Candidate:
    """Temporary blob: not a track until confirmed (reduces ghost tracks).

    ``missed_frames`` counts how many *consecutive* tracking cycles have come and gone
    without a matching detection.  We allow up to ``candidate_max_missed_frames`` (a node
    param) before discarding — without this the candidate dies after a single empty cycle,
    which is fatal when the upstream fusion publish rate is bursty (e.g. paired publishes
    followed by a 200–300 ms silence at km-scale).

    ``pos_history`` stores ``(x, y, z, t_seconds)`` — the explicit timestamp lets the
    velocity computation use the *actual* elapsed time between observations rather than
    the (often wrong) ``CYCLE_PERIOD_S`` proxy.  Without this, an upstream stream that
    publishes every 300 ms gets its velocity inflated 3× (because consecutive history
    points are assumed to be 100 ms apart), which propagates into the seed track velocity
    and makes the freshly-born track diverge from the true target inside ~1 s.

    ``last_update_time`` is the timestamp of the most recent successful match; the
    predictor uses ``now − last_update_time`` directly as the prediction horizon.
    """

    x: float
    y: float
    z: float
    hit_count: int
    pos_history: deque[tuple[float, float, float, float]] = field(
        default_factory=_new_candidate_pos_history,
    )
    missed_frames: int = 0
    last_update_time: float = 0.0


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


def _predicted_candidate_xyz(
    c: Candidate, now: float | None = None,
) -> tuple[float, float, float]:
    """CV prediction for a Candidate using actual time deltas from ``pos_history``.

    With < 2 history points we have no velocity estimate yet, so we return the last position
    (matching the legacy behaviour for the very first match attempt).  Once there are 2+
    points we extrapolate by the *actual elapsed time* since the last update — that horizon
    is ``now - c.last_update_time`` when ``now`` is provided, or it falls back to
    ``(missed_frames + 1) * CYCLE_PERIOD_S`` for callers that do not have a clock (tests).
    Using the real elapsed time eliminates the 2–3× velocity overshoot that occurs when the
    upstream measurement stream is slower than the tracking cycle.
    """
    hist = list(c.pos_history)
    if len(hist) < 2:
        return (c.x, c.y, c.z)
    vx, vy, vz = _initial_velocity_from_history(hist)
    if now is not None and c.last_update_time > 0.0 and now > c.last_update_time:
        horizon = float(now - c.last_update_time)
    else:
        horizon = float(c.missed_frames + 1) * CYCLE_PERIOD_S
    return (c.x + vx * horizon, c.y + vy * horizon, c.z + vz * horizon)


def _initial_velocity_from_history(
    hist: list[tuple[float, float, float, float]],
) -> tuple[float, float, float]:
    """Average per-segment velocity over a (x, y, z, t) position history.

    Each segment uses its *own* ``Δt`` (the difference between consecutive timestamps)
    instead of the cycle period — that way an upstream that publishes every 300 ms does
    not get its velocity inflated 3× by a 100 ms denominator.  Segments with non-positive
    ``Δt`` are skipped defensively (clock glitch / duplicate timestamp).
    """
    if len(hist) < 2:
        return (0.0, 0.0, 0.0)
    sx = sy = sz = 0.0
    n_seg = 0
    for i in range(len(hist) - 1):
        x0, y0, z0, t0 = hist[i]
        x1, y1, z1, t1 = hist[i + 1]
        dt = t1 - t0
        if dt <= 0.0:
            continue
        inv_dt = 1.0 / dt
        sx += (x1 - x0) * inv_dt
        sy += (y1 - y0) * inv_dt
        sz += (z1 - z0) * inv_dt
        n_seg += 1
    if n_seg == 0:
        return (0.0, 0.0, 0.0)
    inv_n = 1.0 / n_seg
    return (sx * inv_n, sy * inv_n, sz * inv_n)


def _distance_det_to_track_gate_pos(det: Point, tr: Track) -> float:
    return _distance_point_to_xyz(det, tr.position_xyz())


def _initial_velocity_from_candidate(cand: Candidate) -> tuple[float, float, float]:
    """Average velocity from consecutive segments: v_k = (p_{k+1} - p_k) / dt.

    Uses up to the last ``CANDIDATE_POS_HISTORY_MAX`` positions. Fallback (0,0,0) if <2 points.
    Implementation delegates to ``_initial_velocity_from_history`` so the candidate-match
    predictor and the confirmation-time ``v_init`` use exactly the same numerical recipe.
    """
    return _initial_velocity_from_history(list(cand.pos_history))


class TrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('tracking_node')
        self.declare_parameter('scenario', 'single')
        self.declare_parameter('max_update_jump_m', 10.0)
        self._max_update_jump_m = float(self.get_parameter('max_update_jump_m').value)
        self.declare_parameter('max_track_speed_mps', 60.0)
        self._max_track_speed_mps = float(self.get_parameter('max_track_speed_mps').value)
        self.declare_parameter('mahalanobis_gate_threshold', 11.34)
        self._mahalanobis_gate_threshold = float(
            self.get_parameter('mahalanobis_gate_threshold').value,
        )
        # Per-cycle gates (meters). The defaults (1 m / 2 m) suit a slow lab toy where target
        # speed * dt << gate, but become *the* track-formation blocker for a km-scale scenario:
        # at 38 m/s LOS and 10 Hz cycle, target travels 3.8 m / frame, so a 1 m candidate gate
        # never confirms and no track is ever born — interceptor sits idle in standby.  Override
        # via config (e.g. config_gazebo_counter_uas.yaml) for fast targets.
        self.declare_parameter('candidate_match_gate_m', CANDIDATE_MERGE_M)
        self._candidate_match_gate_m = float(self.get_parameter('candidate_match_gate_m').value)
        self.declare_parameter('association_gate_m', ASSOCIATION_GATE_M)
        self._association_gate_m = float(self.get_parameter('association_gate_m').value)
        self.declare_parameter('confirmation_hits', CONFIRMATION_HITS)
        self._confirmation_hits = int(self.get_parameter('confirmation_hits').value)
        # When True the candidate-merge gate is checked against a *predicted* candidate
        # position (using its short-history velocity), not the raw last detection.  This makes
        # the gate insensitive to target speed and lets a 1–2 m measurement-noise gate work
        # for both 5 m/s lab targets and 40 m/s long-range targets.
        self.declare_parameter('candidate_predictive_gate', True)
        self._candidate_predictive_gate = bool(self.get_parameter('candidate_predictive_gate').value)
        # Number of *consecutive* tracking cycles a candidate may go without a matching
        # detection before being discarded.  Default 0 reproduces the legacy "one-shot"
        # behaviour (instant discard).  Bump to 3–5 for km-scale where the fused detection
        # stream is bursty (paired publishes followed by 200–300 ms silence) — without
        # tolerance candidates die before they can confirm and no /tracks/state ever fires.
        self.declare_parameter('candidate_max_missed_frames', 0)
        self._candidate_max_missed_frames = int(
            self.get_parameter('candidate_max_missed_frames').value,
        )
        if self._candidate_max_missed_frames < 0:
            self._candidate_max_missed_frames = 0
        # ``primary``: one Point per cycle (lowest track_id) — matches single-target consumers
        # e.g. interception_logic_node. ``all``: legacy multi-publish (breaks those consumers).
        self.declare_parameter('tracks_publish_mode', 'primary')
        _mode = str(self.get_parameter('tracks_publish_mode').value).strip().lower()
        self._tracks_publish_mode = _mode if _mode in ('primary', 'all') else 'primary'
        if self._tracks_publish_mode == 'all':
            self.get_logger().warning(
                'tracks_publish_mode=all publishes multiple Point messages per cycle on /tracks; '
                'nodes that keep only the latest message (e.g. interception_logic_node) will see wrong targets.',
            )
        # Track-state output (with KF velocity + covariance). Uses ``nav_msgs/Odometry`` so we can
        # carry position, velocity, and covariance in one standard message — no custom .msg pkg.
        # ``track_id`` is encoded in ``child_frame_id`` as ``track_<id>`` for downstream selection.
        # Topic / frames default to a sibling stream so legacy ``/tracks`` (Point) keeps working.
        self.declare_parameter('tracks_state_topic', '/tracks/state')
        self.declare_parameter('tracks_state_frame_id', 'map')
        self._tracks_state_topic = str(self.get_parameter('tracks_state_topic').value).strip() or '/tracks/state'
        self._tracks_state_frame_id = str(self.get_parameter('tracks_state_frame_id').value).strip() or 'map'
        self._tracks: list[Track] = []
        self._candidates: list[Candidate] = []
        self._next_id = 1

        self._detection_buffer: list[Point] = []

        dt = CYCLE_PERIOD_S
        self._dt = dt
        self._F = _build_F(dt)
        self._H = _build_H()
        self._Q = _build_Q(dt)
        self._R = _build_R()
        self._Q_miss = _build_Q_miss()

        self._pub = self.create_publisher(Point, '/tracks', 10)
        self._pub_state = self.create_publisher(Odometry, self._tracks_state_topic, 10)
        # Input changed: use fused detections (radar + camera)
        self.create_subscription(Point, '/fused_detections', self._on_detection, 10)
        self._timer = self.create_timer(CYCLE_PERIOD_S, self._on_cycle_timer)
        self.get_logger().info(
            f'Tracking using fused detections (CV Kalman per track); '
            f'/tracks publish mode={self._tracks_publish_mode!r}; '
            f'state topic={self._tracks_state_topic!r} frame={self._tracks_state_frame_id!r}',
        )

    def _on_detection(self, msg: Point) -> None:
        p = Point()
        p.x, p.y, p.z = msg.x, msg.y, msg.z
        self._detection_buffer.append(p)

    def _on_cycle_timer(self) -> None:
        detections = self._detection_buffer
        self._detection_buffer = []
        # Use a single, monotonic "cycle now" so candidate matching, history append, and the
        # predictor all see exactly the same reference time — keeps elapsed-time velocity
        # estimation consistent even if multiple ROS clock ticks happen during the cycle.
        now = self.get_clock().now().nanoseconds * 1e-9
        self._process_cycle(detections, now)

    def _predict_all_tracks(self) -> None:
        for tr in self._tracks:
            tr.kf_predict(self._F, self._Q)
            vx, vy, vz = float(tr.state[3]), float(tr.state[4]), float(tr.state[5])
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            if speed > self._max_track_speed_mps:
                s = self._max_track_speed_mps / speed
                tr.state[3] = vx * s
                tr.state[4] = vy * s
                tr.state[5] = vz * s

    def _process_cycle(self, detections: list[Point], now: float = 0.0) -> None:
        n_old = len(self._tracks)

        # Time update for all existing tracks (prior at this measurement time).
        self._predict_all_tracks()

        # ==================================================================
        # TRACK ASSOCIATION — greedy match with dual gating (same semantics;
        # gate positions taken from KF state **after** predict).
        # ==================================================================
        assoc_pairs: list[tuple[float, float, int, int]] = []
        assoc_gate = self._association_gate_m
        for j, det in enumerate(detections):
            for i, tr in enumerate(self._tracks):
                d_curr = _distance_det_to_track_gate_pos(det, tr)
                pred = tr.gate_ahead_xyz(self._dt)
                d_pred = _distance_point_to_xyz(det, pred)
                if d_curr < assoc_gate and d_pred < assoc_gate:
                    assoc_pairs.append((d_pred, d_curr, i, j))

        assoc_pairs.sort(key=lambda x: (x[0], x[1]))

        matched_track_idx: set[int] = set()
        matched_det_idx: set[int] = set()

        for _d_pred, _d_curr, ti, dj in assoc_pairs:
            if ti in matched_track_idx or dj in matched_det_idx:
                continue
            # Outlier rejection: skip KF update if residual jump is too large.
            tr = self._tracks[ti]
            det = detections[dj]
            pred_xyz = tr.position_xyz()
            rx = float(det.x - pred_xyz[0])
            ry = float(det.y - pred_xyz[1])
            rz = float(det.z - pred_xyz[2])
            residual_dist = math.sqrt(rx * rx + ry * ry + rz * rz)
            if residual_dist > self._max_update_jump_m:
                self.get_logger().info(
                    f'Track {tr.track_id} rejected update (outlier jump {residual_dist:.3f} m > '
                    f'{self._max_update_jump_m:.3f} m)',
                )
                continue
            z = np.array([det.x, det.y, det.z], dtype=float)
            y = z - self._H @ tr.state
            S = self._H @ tr.P @ self._H.T + self._R
            d2 = float(y.T @ np.linalg.inv(S) @ y)
            if d2 > self._mahalanobis_gate_threshold:
                self.get_logger().info(
                    f'Track {tr.track_id} rejected update (Mahalanobis d2={d2:.3f} > '
                    f'{self._mahalanobis_gate_threshold:.3f})',
                )
                continue
            matched_track_idx.add(ti)
            matched_det_idx.add(dj)
            tr.kf_update(det, self._H, self._R)
            vx, vy, vz = float(tr.state[3]), float(tr.state[4]), float(tr.state[5])
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            if speed > self._max_track_speed_mps:
                s = self._max_track_speed_mps / speed
                tr.state[3] = vx * s
                tr.state[4] = vy * s
                tr.state[5] = vz * s
            tr.missed_frames = 0
            tid = tr.track_id
            self.get_logger().info(f'Track {tid} updated (valid match)')

        # Tracks that were not matched: log "bad match" if some detection was
        # close to current OR predicted but not both (prevents silent bad updates).
        for i in range(n_old):
            if i in matched_track_idx:
                continue
            tr = self._tracks[i]
            pred = tr.gate_ahead_xyz(self._dt)
            for det in detections:
                d_curr = _distance_det_to_track_gate_pos(det, tr)
                d_pred = _distance_point_to_xyz(det, pred)
                ok = d_curr < assoc_gate and d_pred < assoc_gate
                partial = (d_curr < assoc_gate or d_pred < assoc_gate) and not ok
                if partial:
                    self.get_logger().info(
                        f'Track {tr.track_id} rejected update (bad match)',
                    )
                    break

        # ==================================================================
        # TRACK LIFECYCLE — miss: no measurement update; inflate P
        # ==================================================================
        for i in range(n_old):
            if i not in matched_track_idx:
                self._tracks[i].kf_miss_inflate(self._Q_miss)
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
        self._process_candidates(unmatched_dets, now)

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

    def _process_candidates(self, unmatched_dets: list[Point], now: float = 0.0) -> None:
        """
        Candidate logic (separate from tracks):

        * Match unmatched detections to existing candidates within CANDIDATE_MERGE_M.
        * Increment hit_count on match; at CONFIRMATION_HITS → real track.
        * Candidates with no matching detection this frame are discarded.
        * Unmatched detections spawn new candidates (hit_count = 1).
        """
        if not unmatched_dets and not self._candidates:
            return

        # When ``candidate_predictive_gate`` is True, match against a one-step CV prediction so
        # the gate stays sized by measurement noise (~ R) instead of target speed * dt.  Without
        # this, fast targets (≳ gate_m / dt) can never accumulate consecutive hits and never
        # get promoted to a track — the symptom is "interceptor doesn't engage" because the
        # /tracks/state stream is permanently empty.
        cand_pairs: list[tuple[float, int, int]] = []
        gate = self._candidate_match_gate_m
        for j, det in enumerate(unmatched_dets):
            for k, cand in enumerate(self._candidates):
                if self._candidate_predictive_gate:
                    pxyz = _predicted_candidate_xyz(cand, now if now > 0.0 else None)
                    d = _distance_point_to_xyz(det, pxyz)
                else:
                    d = _distance_point_to_candidate(det, cand)
                if d < gate:
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
            cand.missed_frames = 0
            # Record the ROS time of this match so the velocity computation uses the
            # *real* elapsed time between consecutive observations, not CYCLE_PERIOD_S.
            t_stamp = now if now > 0.0 else float(cand.last_update_time + CYCLE_PERIOD_S)
            cand.pos_history.append(
                (float(det.x), float(det.y), float(det.z), t_stamp),
            )
            cand.last_update_time = t_stamp

            if cand.hit_count >= self._confirmation_hits:
                tid = self._next_id
                self._next_id += 1
                vx, vy, vz = _initial_velocity_from_candidate(cand)
                self._tracks.append(
                    Track.new_from_position(tid, cand.x, cand.y, cand.z, vx, vy, vz),
                )
                confirmed_indices.add(k)
                self.get_logger().info(
                    f'Candidate confirmed → Track created: track_id={tid} at '
                    f'x={cand.x:.3f}, y={cand.y:.3f}, z={cand.z:.3f} '
                    f'v_init=({vx:.3f},{vy:.3f},{vz:.3f})',
                )

        # Keep matched candidates that were not just promoted to tracks.  Unmatched
        # candidates accrue ``missed_frames`` and are only dropped when the counter
        # exceeds ``candidate_max_missed_frames`` — see the param block in __init__
        # for the rationale (bursty fusion stream at km-scale).
        new_candidates: list[Candidate] = []
        for k, cand in enumerate(self._candidates):
            if k in confirmed_indices:
                continue
            if k in matched_cand_idx:
                new_candidates.append(cand)
                continue
            cand.missed_frames += 1
            if cand.missed_frames > self._candidate_max_missed_frames:
                self.get_logger().info(
                    f'Candidate discarded: position=({cand.x:.3f}, {cand.y:.3f}, {cand.z:.3f}) '
                    f'hit_count was {cand.hit_count} '
                    f'missed_frames={cand.missed_frames}',
                )
                continue
            new_candidates.append(cand)

        self._candidates = new_candidates

        # Spawn new candidates for detections that did not match any candidate.
        t_stamp_new = now if now > 0.0 else 0.0
        for j, det in enumerate(unmatched_dets):
            if j in matched_unmatched_det_idx:
                continue
            nc = Candidate(det.x, det.y, det.z, 1)
            nc.pos_history.append(
                (float(det.x), float(det.y), float(det.z), t_stamp_new),
            )
            nc.last_update_time = t_stamp_new
            self._candidates.append(nc)
            self.get_logger().info(
                f'Candidate detected: position=({det.x:.3f}, {det.y:.3f}, {det.z:.3f}) '
                f'hit_count=1',
            )

    def _publish_all_tracks(self) -> None:
        if not self._tracks:
            return
        if self._tracks_publish_mode == 'primary':
            tr = min(self._tracks, key=lambda t: t.track_id)
            out = Point()
            out.x = tr.x
            out.y = tr.y
            out.z = tr.z
            self._pub.publish(out)
            self._pub_state.publish(self._track_to_odometry(tr))
            return
        # Legacy multi-publish path: also emit one Odometry per track for downstream consumers
        # that *do* support multi-target on /tracks/state (none today, but keeps the contract honest).
        for tr in self._tracks:
            out = Point()
            out.x = tr.x
            out.y = tr.y
            out.z = tr.z
            self._pub.publish(out)
            self._pub_state.publish(self._track_to_odometry(tr))

    def _track_to_odometry(self, tr: Track) -> Odometry:
        """Pack a ``Track`` into ``nav_msgs/Odometry`` with KF state + 6x6 pose/twist covariance.

        ``child_frame_id`` carries the integer track id as ``track_<id>`` so consumers can
        select a specific track without a custom message type.  Orientation is identity
        (CV model does not estimate attitude); rotational covariances are large to signal
        "unknown" without breaking PoseWithCovariance consumers.
        """
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._tracks_state_frame_id
        msg.child_frame_id = f'track_{tr.track_id}'
        msg.pose.pose.position.x = float(tr.state[0])
        msg.pose.pose.position.y = float(tr.state[1])
        msg.pose.pose.position.z = float(tr.state[2])
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = float(tr.state[3])
        msg.twist.twist.linear.y = float(tr.state[4])
        msg.twist.twist.linear.z = float(tr.state[5])
        # 6x6 pose covariance (row-major, [x, y, z, rx, ry, rz]).
        # Position block from the 3x3 KF position covariance, rotational block large = unknown.
        pose_cov = [0.0] * 36
        for i in range(3):
            for j in range(3):
                pose_cov[i * 6 + j] = float(tr.P[i, j])
        for k in range(3, 6):
            pose_cov[k * 6 + k] = 1e6
        msg.pose.covariance = pose_cov
        # 6x6 twist covariance (linear xyz + angular xyz). Linear block from KF velocity covariance.
        twist_cov = [0.0] * 36
        for i in range(3):
            for j in range(3):
                twist_cov[i * 6 + j] = float(tr.P[3 + i, 3 + j])
        for k in range(3, 6):
            twist_cov[k * 6 + k] = 1e6
        msg.twist.covariance = twist_cov
        return msg

    def _log_active_tracks(self) -> None:
        ids = [tr.track_id for tr in self._tracks]
        self.get_logger().info(
            f'Active tracks: {len(self._tracks)} — track_ids: {ids}',
        )

    def _log_all_tracks(self) -> None:
        for tr in self._tracks:
            vx, vy, vz = float(tr.state[3]), float(tr.state[4]), float(tr.state[5])
            self.get_logger().info(
                f'Track {tr.track_id}: x={tr.x:.3f}, y={tr.y:.3f}, z={tr.z:.3f} '
                f'v=({vx:.3f},{vy:.3f},{vz:.3f}) (missed_frames={tr.missed_frames})',
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
