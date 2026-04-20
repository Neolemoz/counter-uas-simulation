"""Multi-interceptor: hysteresis TTI reassignment + predictive intercept + PN on the committed unit only."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_msgs.msg import Bool

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


def _norm(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _dot(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> float:
    return ax * bx + ay * by + az * bz


def _cross(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> tuple[float, float, float]:
    return (
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx,
    )


def _unit(dx: float, dy: float, dz: float, eps: float = 1e-9) -> tuple[float, float, float]:
    n = _norm(dx, dy, dz)
    if n < eps:
        return (0.0, 0.0, 0.0)
    return (dx / n, dy / n, dz / n)


def _solve_intercept_time(
    p_tx: float,
    p_ty: float,
    p_tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    p_ix: float,
    p_iy: float,
    p_iz: float,
    s_i: float,
) -> float | None:
    r0x = p_tx - p_ix
    r0y = p_ty - p_iy
    r0z = p_tz - p_iz
    vv = _dot(v_tx, v_ty, v_tz, v_tx, v_ty, v_tz)
    rv = _dot(r0x, r0y, r0z, v_tx, v_ty, v_tz)
    rr = _dot(r0x, r0y, r0z, r0x, r0y, r0z)

    a = vv - s_i * s_i
    b = 2.0 * rv
    c = rr
    eps = 1e-12
    candidates: list[float] = []

    if abs(a) < eps:
        if abs(b) < eps:
            return None
        t_lin = -c / b
        if t_lin > 0.0:
            candidates.append(t_lin)
    else:
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None
        sqrt_d = math.sqrt(disc)
        for t in ((-b - sqrt_d) / (2.0 * a), (-b + sqrt_d) / (2.0 * a)):
            if t > 0.0:
                candidates.append(float(t))

    if not candidates:
        return None

    valid: list[float] = []
    for t in candidates:
        hx = p_tx + v_tx * t - p_ix
        hy = p_ty + v_ty * t - p_iy
        hz = p_tz + v_tz * t - p_iz
        lhs = _norm(hx, hy, hz)
        rhs = s_i * t
        if abs(lhs - rhs) <= 1e-6 * max(1.0, lhs, rhs):
            valid.append(t)

    if not valid:
        return None
    return min(valid)


def _compute_intercept(
    p_tx: float,
    p_ty: float,
    p_tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    p_ix: float,
    p_iy: float,
    p_iz: float,
    s_i: float,
) -> tuple[float, float, float, float, float, float, float] | None:
    t = _solve_intercept_time(p_tx, p_ty, p_tz, v_tx, v_ty, v_tz, p_ix, p_iy, p_iz, s_i)
    if t is None or not math.isfinite(t):
        return None
    phx = p_tx + v_tx * t
    phy = p_ty + v_ty * t
    phz = p_tz + v_tz * t
    ux, uy, uz = _unit(phx - p_ix, phy - p_iy, phz - p_iz)
    if _norm(ux, uy, uz) < 1e-9:
        return None
    return (t, phx, phy, phz, ux, uy, uz)


def _pn_steering_vector(
    rx: float,
    ry: float,
    rz: float,
    v_rx: float,
    v_ry: float,
    v_rz: float,
    pn_n: float,
    min_vc: float,
) -> tuple[float, float, float, float, bool]:
    rn = _norm(rx, ry, rz)
    if rn < 1e-6:
        return (0.0, 0.0, 0.0, 0.0, False)
    rhx, rhy, rhz = rx / rn, ry / rn, rz / rn
    inv_r2 = 1.0 / (rn * rn + 1e-12)
    lrx, lry, lrz = _cross(rx, ry, rz, v_rx, v_ry, v_rz)
    lrx *= inv_r2
    lry *= inv_r2
    lrz *= inv_r2
    vc = -_dot(rhx, rhy, rhz, v_rx, v_ry, v_rz)
    if vc < min_vc:
        return (0.0, 0.0, 0.0, vc, False)
    sx, sy, sz = _cross(lrx, lry, lrz, rhx, rhy, rhz)
    sx *= pn_n * vc
    sy *= pn_n * vc
    sz *= pn_n * vc
    if _norm(sx, sy, sz) < 1e-12:
        return (0.0, 0.0, 0.0, vc, False)
    return (sx, sy, sz, vc, True)


def _tti_feasible(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    closing: float,
    t_min: float,
    t_max: float,
) -> tuple[bool, float | None]:
    sol = _compute_intercept(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, closing)
    if sol is None:
        return (False, None)
    t_hit, _phx, _phy, _phz, ux, uy, uz = sol
    if not (t_min <= t_hit <= t_max) or _norm(ux, uy, uz) < 1e-6:
        return (False, None)
    return (True, float(t_hit))


class InterceptionLogicNode(Node):
    """
    Subscribes to ``/drone/position`` and ``/<id>/position`` for each interceptor id.
    Maintains a **committed** interceptor; reassignment uses TTI margin + minimum dwell
    ``switch_window_s`` (seconds since last commit). Forced reassignment when the committed
    unit becomes infeasible. Publishes ``Vector3`` on ``/<id>/cmd_velocity`` (non-committed = 0).
    """

    def __init__(self) -> None:
        super().__init__('interception_logic_node')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('closing_speed_m_s', 4.5)
        self.declare_parameter('max_speed_m_s', 7.5)
        self.declare_parameter('min_distance_m', 0.45)
        self.declare_parameter('log_period_s', 1.0)
        self.declare_parameter('selection_log_period_s', 2.0)
        self.declare_parameter('selection_margin_s', 0.3)
        self.declare_parameter('switch_window_s', 2.0)
        self.declare_parameter('lost_timeout_s', 1.5)
        self.declare_parameter('reacquire_confirm_s', 0.25)
        self.declare_parameter('max_intercept_time_s', 90.0)
        self.declare_parameter('min_intercept_time_s', 0.02)
        self.declare_parameter('target_topic', '/drone/position')
        self.declare_parameter('selected_id_topic', '/interceptor/selected_id')
        self.declare_parameter('lock_selected_after_first', True)
        self.declare_parameter('hit_threshold_m', 1.0)
        self.declare_parameter('stop_topic', '/target/stop')
        self.declare_parameter('interceptor_ids', ['interceptor_0', 'interceptor_1', 'interceptor_2'])
        self.declare_parameter('use_pn_refinement', True)
        self.declare_parameter('pn_navigation_constant', 3.0)
        self.declare_parameter('pn_blend_gain', 0.22)
        self.declare_parameter('pn_min_closing_speed_m_s', 0.15)

        self._closing = max(float(self.get_parameter('closing_speed_m_s').value), 0.1)
        self._vmax = max(float(self.get_parameter('max_speed_m_s').value), self._closing)
        self._r_stop = max(float(self.get_parameter('min_distance_m').value), 0.05)
        self._log_period = max(float(self.get_parameter('log_period_s').value), 0.2)
        self._sel_log_period = max(float(self.get_parameter('selection_log_period_s').value), 0.5)
        self._t_hit_max = max(float(self.get_parameter('max_intercept_time_s').value), 0.5)
        self._t_hit_min = max(float(self.get_parameter('min_intercept_time_s').value), 1e-4)
        self._use_pn = bool(self.get_parameter('use_pn_refinement').value)
        self._pn_n = max(float(self.get_parameter('pn_navigation_constant').value), 0.1)
        self._pn_blend = max(float(self.get_parameter('pn_blend_gain').value), 0.0)
        self._pn_min_vc = max(float(self.get_parameter('pn_min_closing_speed_m_s').value), 0.0)
        self._tti_margin = max(float(self.get_parameter('selection_margin_s').value), 0.0)
        self._switch_window_s = max(float(self.get_parameter('switch_window_s').value), 0.0)
        self._lost_timeout_s = max(float(self.get_parameter('lost_timeout_s').value), 0.0)
        self._reacquire_confirm_s = max(float(self.get_parameter('reacquire_confirm_s').value), 0.0)

        raw_ids = self.get_parameter('interceptor_ids').value
        if isinstance(raw_ids, list) and raw_ids:
            self._ids = [str(x).strip() for x in raw_ids if str(x).strip()]
        else:
            self._ids = ['interceptor_0', 'interceptor_1', 'interceptor_2']

        tgt_topic = str(self.get_parameter('target_topic').value).strip()
        sel_topic = str(self.get_parameter('selected_id_topic').value).strip()
        stop_topic = str(self.get_parameter('stop_topic').value).strip()
        rate = max(float(self.get_parameter('rate_hz').value), 1.0)
        self._lock_after_first = bool(self.get_parameter('lock_selected_after_first').value)
        self._hit_thresh = max(float(self.get_parameter('hit_threshold_m').value), 0.05)

        self._pubs: dict[str, Publisher] = {}
        for iid in self._ids:
            cmd_topic = f'/{iid}/cmd_velocity'
            self._pubs[iid] = self.create_publisher(Vector3, cmd_topic, 10)
        self._pub_selected = self.create_publisher(String, sel_topic, 10)
        self._pub_stop = self.create_publisher(Bool, stop_topic, 10)

        self.create_subscription(Point, tgt_topic, self._on_target, 10)
        for iid in self._ids:
            pos_topic = f'/{iid}/position'
            self.create_subscription(
                Point,
                pos_topic,
                lambda msg, name=iid: self._on_inter(name, msg),
                10,
            )

        self._target: Point | None = None
        self._inter_pos: dict[str, Point | None] = {i: None for i in self._ids}
        self._prev_target: tuple[float, float, float] | None = None
        self._prev_target_time: Time | None = None
        self._prev_inter_pos: dict[str, tuple[float, float, float] | None] = {i: None for i in self._ids}
        self._prev_inter_time: dict[str, Time | None] = {i: None for i in self._ids}

        self._last_log = self.get_clock().now()
        self._last_sel_log = self.get_clock().now()
        self._t0 = self.get_clock().now()
        self._current_selected_id: str | None = None
        self._committed_since: Time | None = None
        self._lost_since: Time | None = None
        self._reacquire_since: Time | None = None
        self._switch_count = 0
        self._best_id_last: str | None = None
        self._last_hold_log: Time | None = None
        self._locked_selected_id: str | None = None
        self._hit = False

        period = 1.0 / rate
        self._timer = self.create_timer(period, self._on_control)
        self.get_logger().info(
            f'Multi-interception (predict + PN, TTI+hysteresis): {tgt_topic} + '
            f'{[f"/{i}/position" for i in self._ids]} -> cmd per id; '
            f'margin={self._tti_margin}s window={self._switch_window_s}s lost_timeout={self._lost_timeout_s}s '
            f'({rate:.0f} Hz)',
        )

    def _on_target(self, msg: Point) -> None:
        self._target = msg

    def _on_inter(self, iid: str, msg: Point) -> None:
        self._inter_pos[iid] = msg

    def _estimate_target_vel(self, tx: float, ty: float, tz: float) -> tuple[float, float, float]:
        now = self.get_clock().now()
        if self._prev_target is None or self._prev_target_time is None:
            self._prev_target = (tx, ty, tz)
            self._prev_target_time = now
            return (0.0, 0.0, 0.0)
        dt = (now - self._prev_target_time).nanoseconds * 1e-9
        px, py, pz = self._prev_target
        self._prev_target = (tx, ty, tz)
        self._prev_target_time = now
        if dt < 1e-3:
            return (0.0, 0.0, 0.0)
        return ((tx - px) / dt, (ty - py) / dt, (tz - pz) / dt)

    def _estimate_interceptor_vel(
        self,
        iid: str,
        ix: float,
        iy: float,
        iz: float,
    ) -> tuple[float, float, float]:
        now = self.get_clock().now()
        prev = self._prev_inter_pos[iid]
        prev_t = self._prev_inter_time[iid]
        if prev is None or prev_t is None:
            self._prev_inter_pos[iid] = (ix, iy, iz)
            self._prev_inter_time[iid] = now
            return (0.0, 0.0, 0.0)
        dt = (now - prev_t).nanoseconds * 1e-9
        px, py, pz = prev
        self._prev_inter_pos[iid] = (ix, iy, iz)
        self._prev_inter_time[iid] = now
        if dt < 1e-3:
            return (0.0, 0.0, 0.0)
        return ((ix - px) / dt, (iy - py) / dt, (iz - pz) / dt)

    def _clamp_speed(self, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
        n = _norm(vx, vy, vz)
        if n < 1e-9 or n <= self._vmax:
            return (vx, vy, vz)
        s = self._vmax / n
        return (vx * s, vy * s, vz * s)

    def _publish_all(self, velocities: dict[str, Vector3]) -> None:
        for iid in self._ids:
            self._pubs[iid].publish(velocities.get(iid, Vector3()))

    def _time_since_commit_s(self, now: Time) -> float:
        if self._committed_since is None:
            return float('inf')
        return (now - self._committed_since).nanoseconds * 1e-9

    def _apply_commit_change(self, old: str | None, new: str | None, now: Time) -> None:
        if old == new:
            return
        t_rel = (now - self._t0).nanoseconds * 1e-9
        if old is not None and new is not None:
            print(f'[SWITCH t={t_rel:.3f}s] {old} -> {new}', flush=True)
            self._switch_count += 1
        elif old is not None and new is None:
            print(f'[SWITCH t={t_rel:.3f}s] {old} -> (none)', flush=True)
            self._switch_count += 1
        self._current_selected_id = new
        self._committed_since = now

    def _maybe_hold_log(self, now: Time, msg: str) -> None:
        # Avoid spamming HOLD every control tick.
        if self._last_hold_log is None:
            self._last_hold_log = now
            print(msg, flush=True)
            return
        age = (now - self._last_hold_log).nanoseconds * 1e-9
        if age >= 0.5:
            self._last_hold_log = now
            print(msg, flush=True)

    def _update_committed_selection(
        self,
        rows: list[tuple[str, bool, float | None]],
        now: Time,
    ) -> str | None:
        tti_by = {iid: (fe, tti) for iid, fe, tti in rows}
        feasible_cands = [(tti, iid) for iid, (fe, tti) in tti_by.items() if fe and tti is not None]
        best_id: str | None = None
        if feasible_cands:
            best_id = min(feasible_cands, key=lambda x: (x[0], self._ids.index(x[1])))[1]
        self._best_id_last = best_id

        cur = self._current_selected_id

        # Lock selection after first feasible pick so only one interceptor will ever be commanded/launched.
        if self._lock_after_first and self._locked_selected_id is not None:
            self._current_selected_id = self._locked_selected_id
            return self._current_selected_id

        if cur is None:
            # If any interceptor is feasible, never commit to (none).
            self._apply_commit_change(None, best_id, now)
            if self._lock_after_first and self._current_selected_id is not None:
                self._locked_selected_id = self._current_selected_id
            return self._current_selected_id

        c_feas, c_tti = tti_by.get(cur, (False, None))
        any_feasible = best_id is not None

        # If the committed interceptor is currently infeasible, apply a grace period before dropping it.
        # Do not clear the lost timer on a single transient feasible tick; require reacquire_confirm_s.
        if not c_feas or c_tti is None:
            self._reacquire_since = None
            if self._lost_since is None:
                self._lost_since = now
            lost_age = (now - self._lost_since).nanoseconds * 1e-9

            if any_feasible:
                # Prefer to keep current during transient infeasible periods.
                if lost_age < self._lost_timeout_s:
                    self._maybe_hold_log(
                        now,
                        f'[HOLD] keeping {cur} despite temporary infeasible (lost_age={lost_age:.2f}s)',
                    )
                    return cur
                # After timeout, switch to best feasible (never to none if any feasible).
                self._apply_commit_change(cur, best_id, now)
                self._lost_since = None
                return self._current_selected_id

            # No feasible interceptors at all: allow (none) only after timeout.
            if lost_age < self._lost_timeout_s:
                self._maybe_hold_log(
                    now,
                    f'[HOLD] keeping {cur} despite all-infeasible (lost_age={lost_age:.2f}s)',
                )
                return cur
            self._apply_commit_change(cur, None, now)
            self._lost_since = None
            return self._current_selected_id

        # Committed is feasible again -> reset lost timer only after reacquire_confirm_s.
        if self._lost_since is not None and self._reacquire_confirm_s > 0.0:
            if self._reacquire_since is None:
                self._reacquire_since = now
            reacq_age = (now - self._reacquire_since).nanoseconds * 1e-9
            if reacq_age >= self._reacquire_confirm_s:
                self._lost_since = None
                self._reacquire_since = None
        else:
            self._lost_since = None
            self._reacquire_since = None

        new_id = cur
        if best_id is not None and best_id != cur:
            b_feas, b_tti = tti_by[best_id]
            dwell = self._time_since_commit_s(now)
            if (
                b_feas
                and b_tti is not None
                and c_tti is not None
                and b_tti + self._tti_margin < c_tti
                and dwell >= self._switch_window_s
            ):
                new_id = best_id

        self._apply_commit_change(cur, new_id, now)
        if self._lock_after_first and self._current_selected_id is not None and self._locked_selected_id is None:
            self._locked_selected_id = self._current_selected_id
        return self._current_selected_id

    def _on_control(self) -> None:
        zero = Vector3()
        if self._target is None:
            self._publish_all({i: zero for i in self._ids})
            return

        tx, ty, tz = float(self._target.x), float(self._target.y), float(self._target.z)
        v_tx, v_ty, v_tz = self._estimate_target_vel(tx, ty, tz)

        rows: list[tuple[str, bool, float | None]] = []
        for iid in self._ids:
            p = self._inter_pos.get(iid)
            if p is None:
                rows.append((iid, False, None))
                continue
            ix, iy, iz = float(p.x), float(p.y), float(p.z)
            ok, tti = _tti_feasible(
                tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
                self._closing, self._t_hit_min, self._t_hit_max,
            )
            rows.append((iid, ok, tti))

        now = self.get_clock().now()
        selected = self._update_committed_selection(rows, now)

        # Publish committed selected id for interceptor launch gating.
        sel_msg = String()
        sel_msg.data = selected if selected is not None else ''
        self._pub_selected.publish(sel_msg)

        self._maybe_selection_log(rows, selected)

        if selected is None:
            self._publish_all({i: zero for i in self._ids})
            return

        p_sel = self._inter_pos.get(selected)
        if p_sel is None:
            self._publish_all({i: zero for i in self._ids})
            return

        ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
        dist_to_target = _norm(tx - ix, ty - iy, tz - iz)

        # HIT detection (one-shot): stop target motion and freeze interceptor commands.
        if (not self._hit) and dist_to_target < self._hit_thresh:
            self._hit = True
            print(f'[HIT] {selected}', flush=True)
            stop = Bool()
            stop.data = True
            self._pub_stop.publish(stop)
            self._publish_all({i: zero for i in self._ids})
            return

        if self._hit:
            # After hit: keep everything stopped.
            stop = Bool()
            stop.data = True
            self._pub_stop.publish(stop)
            self._publish_all({i: zero for i in self._ids})
            return

        v_ix, v_iy, v_iz = self._estimate_interceptor_vel(selected, ix, iy, iz)

        dist = dist_to_target
        out_map = {i: Vector3() for i in self._ids}

        if dist < self._r_stop:
            self._publish_all(out_map)
            self._maybe_detail_log(tx, ty, tz, ix, iy, iz, dist, None, None, None, 0.0, 0.0, 0.0, 'hold', False, 0.0, selected)
            return

        mode = 'naive'
        t_hit: float | None = None
        phx = phy = phz = 0.0
        ux = uy = uz = 0.0

        sol = _compute_intercept(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, self._closing)
        if sol is not None:
            t_hit, phx, phy, phz, ux, uy, uz = sol
            if self._t_hit_min <= t_hit <= self._t_hit_max and _norm(ux, uy, uz) > 1e-6:
                mode = 'predict'
            else:
                sol = None

        if sol is None or mode != 'predict':
            ux, uy, uz = _unit(tx - ix, ty - iy, tz - iz)
            t_hit = None
            phx = phy = phz = float('nan')

        pn_active = False
        vc_log = 0.0
        if self._use_pn and self._pn_blend > 1e-9 and dist > 1e-6:
            rx, ry, rz = tx - ix, ty - iy, tz - iz
            v_rx = v_tx - v_ix
            v_ry = v_ty - v_iy
            v_rz = v_tz - v_iz
            sx, sy, sz, vc_log, pn_ok = _pn_steering_vector(
                rx, ry, rz, v_rx, v_ry, v_rz, self._pn_n, self._pn_min_vc,
            )
            if pn_ok:
                bx = ux + self._pn_blend * sx
                by = uy + self._pn_blend * sy
                bz = uz + self._pn_blend * sz
                un = _norm(bx, by, bz)
                if un > 1e-9:
                    ux, uy, uz = bx / un, by / un, bz / un
                    pn_active = True

        vx = ux * self._closing
        vy = uy * self._closing
        vz = uz * self._closing
        vx, vy, vz = self._clamp_speed(vx, vy, vz)
        cmd = Vector3()
        cmd.x, cmd.y, cmd.z = vx, vy, vz
        out_map[selected] = cmd
        self._publish_all(out_map)
        self._maybe_detail_log(
            tx, ty, tz, ix, iy, iz, dist, t_hit, phx, phy, phz, vx, vy, vz, mode, pn_active, vc_log, selected,
        )

    def _maybe_selection_log(self, rows: list[tuple[str, bool, float | None]], selected: str | None) -> None:
        now = self.get_clock().now()
        if (now - self._last_sel_log).nanoseconds * 1e-9 < self._sel_log_period:
            return
        self._last_sel_log = now
        lines = ['=== Interceptor Selection ===']
        for iid, fe, tti in rows:
            if tti is None:
                t_s = 'n/a'
            else:
                t_s = f'{tti:.3f}'
            lines.append(f'{iid}: feasible={fe}, tti={t_s}')
        lines.append(f'selected: {selected if selected is not None else "(none)"}')
        lines.append(f'best_tti: {self._best_id_last if self._best_id_last is not None else "(none)"}')
        lines.append(f'switch_count: {self._switch_count}')
        # Use ROS logger so the table reliably appears in `ros2 launch` output.
        self.get_logger().info('\n'.join(lines))

    def _maybe_detail_log(
        self,
        tx: float,
        ty: float,
        tz: float,
        ix: float,
        iy: float,
        iz: float,
        dist: float,
        t_hit: float | None,
        phx: float,
        phy: float,
        phz: float,
        vx: float,
        vy: float,
        vz: float,
        mode: str,
        pn_active: bool,
        vc: float,
        selected: str,
    ) -> None:
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds * 1e-9 < self._log_period:
            return
        self._last_log = now
        if t_hit is not None and math.isfinite(phx):
            phs = f'({phx:.2f},{phy:.2f},{phz:.2f})'
            ts = f'{t_hit:.2f}'
        else:
            phs = '(n/a)'
            ts = 'n/a'
        pn_s = 'on' if pn_active else 'off'
        print(
            f'[interception] sel={selected} mode={mode} pn={pn_s} dist={dist:.2f} m | t_hit={ts} s p_hit={phs} | '
            f'Vc={vc:.2f} m/s | target=({tx:.2f},{ty:.2f},{tz:.2f}) inter=({ix:.2f},{iy:.2f},{iz:.2f}) | '
            f'cmd_vel=({vx:.2f},{vy:.2f},{vz:.2f})',
            flush=True,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InterceptionLogicNode()
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
