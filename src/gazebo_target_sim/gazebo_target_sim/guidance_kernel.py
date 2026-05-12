"""Deterministic kinematic guidance command kernel.

This module is intentionally ROS-free.  It implements the live guidance command
assembly used by ``interception_logic_node``: CV predictive intercept, pursuit
fallback, terminal pursuit blending, PN-inspired steering-vector blending,
unit-vector slew limiting, and speed/t_go coherence.  It does not implement
assignment, dome policy, hit policy, Gazebo plant limiting, or visualization.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from gazebo_target_sim.guidance_lib import (
    compensate_target_for_delay,
    compute_intercept,
    norm,
    unit,
)

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class TargetKinematics:
    position: Vec3
    velocity: Vec3
    measurement_delay_s: float = 0.0


@dataclass(frozen=True)
class InterceptorKinematics:
    position: Vec3
    velocity: Vec3


@dataclass(frozen=True)
class GuidanceParams:
    interceptor_max_speed_m_s: float
    max_speed_m_s: float
    speed_k1: float
    speed_k2: float
    speed_vmin_m_s: float
    speed_tgo_min_s: float
    t_hit_min_s: float
    t_hit_max_s: float
    pursuit_lead_blend: float
    naive_lead_time_s: float
    predict_enter_frames: int
    predict_exit_frames: int
    intercept_smoothing_alpha: float
    align_speed_to_solver: bool
    t_go_filter_alpha: float
    t_go_filter_max_step_s: float
    align_speed_use_smooth_hit_range: bool
    guidance_u_max_step_rad: float
    guidance_terminal_range_m: float
    guidance_terminal_pursuit_blend: float
    use_pn_inspired: bool
    pn_navigation_constant: float
    pn_blend_gain: float
    pn_blend_terminal_range_m: float
    pn_min_closing_speed_m_s: float
    debug_speed_margin_m_s: float = 0.0


@dataclass(frozen=True)
class GuidanceMemory:
    mode: str = "pursuit"
    predict_valid_count: int = 0
    predict_invalid_count: int = 0
    filtered_t_go_s: float | None = None
    smoothed_intercept_point: Vec3 | None = None
    previous_unit_command: Vec3 | None = None


@dataclass(frozen=True)
class GuidanceInput:
    target: TargetKinematics
    interceptor: InterceptorKinematics
    params: GuidanceParams
    memory: GuidanceMemory
    aim_point_override: Vec3 | None = None
    debug_predictive_override: bool = False


@dataclass(frozen=True)
class GuidanceCommand:
    velocity_cmd: Vec3
    unit_cmd: Vec3
    speed_cmd_m_s: float
    mode: str
    solution_valid: bool
    t_go_raw_s: float | None
    t_go_effective_s: float | None
    intercept_point_raw: Vec3 | None
    intercept_point_smoothed: Vec3 | None
    solver_speed_m_s: float | None
    pursuit_unit: Vec3
    predictive_unit: Vec3 | None
    pn_inspired_active: bool
    closing_speed_m_s: float | None
    memory_next: GuidanceMemory
    mode_transition: tuple[str, str] | None = None
    debug_predictive_override_active: bool = False
    debug_required_speed_m_s: float | None = None


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _norm_vec(a: Vec3) -> float:
    return norm(a[0], a[1], a[2])


def _unit_vec(a: Vec3) -> Vec3:
    return unit(a[0], a[1], a[2])


def _command_speed(params: GuidanceParams, dist: float, t_go: float | None = None) -> float:
    if not math.isfinite(dist) or dist <= 0.0:
        return float(params.speed_vmin_m_s)
    if t_go is not None and math.isfinite(t_go) and t_go > 0.0:
        t_eff = max(float(t_go), params.speed_tgo_min_s)
        v = params.speed_k1 * dist + params.speed_k2 * dist / t_eff
    else:
        v = params.speed_k1 * dist
    return float(min(params.max_speed_m_s, max(params.speed_vmin_m_s, v)))


def _clamp_speed(params: GuidanceParams, v: Vec3) -> Vec3:
    n = _norm_vec(v)
    if n < 1e-9 or n <= params.max_speed_m_s:
        return v
    return _scale(v, params.max_speed_m_s / n)


def _compute_required_speed(p_i: Vec3, p_hit: Vec3, t_go: float) -> float:
    if t_go <= 0.0 or not math.isfinite(t_go):
        return float("inf")
    d = _norm_vec(_sub(p_hit, p_i))
    if not math.isfinite(d):
        return float("inf")
    return d / t_go


def rotate_dir_toward(a: Vec3, b: Vec3, max_angle: float) -> Vec3:
    ax, ay, az = _unit_vec(a)
    bx, by, bz = _unit_vec(b)
    if _norm_vec((ax, ay, az)) < 1e-9:
        return (bx, by, bz)
    if _norm_vec((bx, by, bz)) < 1e-9:
        return (ax, ay, az)
    dot = max(-1.0, min(1.0, _dot((ax, ay, az), (bx, by, bz))))
    angle = math.acos(dot)
    if angle <= max_angle or angle < 1e-9:
        return (bx, by, bz)
    t = max_angle / angle
    sin_a = math.sin(angle)
    if sin_a < 1e-9:
        return (bx, by, bz)
    fa = math.sin((1.0 - t) * angle) / sin_a
    fb = math.sin(t * angle) / sin_a
    r = (fa * ax + fb * bx, fa * ay + fb * by, fa * az + fb * bz)
    rn = _norm_vec(r)
    if rn < 1e-9:
        return (ax, ay, az)
    return (r[0] / rn, r[1] / rn, r[2] / rn)


def pn_inspired_steering_vector(
    r: Vec3,
    v_rel: Vec3,
    navigation_constant: float,
    min_closing_speed_m_s: float,
) -> tuple[Vec3, float, bool]:
    """Return the live PN-inspired steering vector, closing speed, and active flag.

    This is not a classical acceleration-command PN controller.  The vector is
    later blended into the commanded unit direction and then interpreted by the
    kinematic velocity-command path.
    """
    rn = _norm_vec(r)
    if rn < 1e-6:
        return ((0.0, 0.0, 0.0), 0.0, False)
    r_hat = (r[0] / rn, r[1] / rn, r[2] / rn)
    inv_r2 = 1.0 / (rn * rn + 1e-12)
    los_rate = _scale(_cross(r, v_rel), inv_r2)
    vc = -_dot(r_hat, v_rel)
    if vc < min_closing_speed_m_s:
        return ((0.0, 0.0, 0.0), vc, False)
    steer = _scale(_cross(los_rate, r_hat), navigation_constant * vc)
    if _norm_vec(steer) < 1e-12:
        return ((0.0, 0.0, 0.0), vc, False)
    return (steer, vc, True)


def effective_pn_inspired_blend(params: GuidanceParams, dist_to_target: float) -> float:
    if params.pn_blend_terminal_range_m <= 1e-6:
        return params.pn_blend_gain
    scale = max(0.0, min(1.0, 1.0 - float(dist_to_target) / params.pn_blend_terminal_range_m))
    return params.pn_blend_gain * scale


def update_guidance_mode(
    memory: GuidanceMemory,
    params: GuidanceParams,
    solution_valid: bool,
) -> tuple[str, int, int, float, tuple[str, str] | None]:
    valid = memory.predict_valid_count
    invalid = memory.predict_invalid_count
    if solution_valid:
        valid = min(valid + 1, max(params.predict_enter_frames, 1))
        invalid = 0
    else:
        invalid = min(invalid + 1, max(params.predict_exit_frames, 1))
        valid = 0

    prev_mode = memory.mode if memory.mode in ("pursuit", "predict") else "pursuit"
    mode = prev_mode
    transition: tuple[str, str] | None = None
    if prev_mode == "pursuit" and valid >= max(params.predict_enter_frames, 1):
        mode = "predict"
        transition = ("pursuit", "predict")
    elif prev_mode == "predict" and invalid >= max(params.predict_exit_frames, 1):
        mode = "pursuit"
        transition = ("predict", "pursuit")

    if mode == "predict":
        blend = 1.0 - invalid / max(params.predict_exit_frames, 1)
    else:
        blend = valid / max(params.predict_enter_frames, 1)
    return mode, valid, invalid, max(0.0, min(1.0, blend)), transition


def smooth_intercept_point(
    previous: Vec3 | None,
    raw: Vec3,
    alpha: float,
) -> Vec3:
    if previous is None:
        return raw
    a = max(0.0, min(1.0, float(alpha)))
    return (
        a * raw[0] + (1.0 - a) * previous[0],
        a * raw[1] + (1.0 - a) * previous[1],
        a * raw[2] + (1.0 - a) * previous[2],
    )


def _terminal_adjusted_blend(params: GuidanceParams, dist: float, blend: float) -> float:
    if (
        params.guidance_terminal_range_m > 1e-6
        and dist > 1e-9
        and params.guidance_terminal_pursuit_blend > 1e-9
    ):
        frac = max(0.0, min(1.0, 1.0 - dist / params.guidance_terminal_range_m))
        blend *= 1.0 - params.guidance_terminal_pursuit_blend * frac
    return max(0.0, min(1.0, blend))


def _pursuit_unit(target_pos: Vec3, target_vel: Vec3, aim_point: Vec3, interceptor_pos: Vec3, params: GuidanceParams) -> Vec3:
    los = _unit_vec(_sub(aim_point, interceptor_pos))
    bl = params.pursuit_lead_blend
    speed_sum = abs(target_vel[0]) + abs(target_vel[1]) + abs(target_vel[2])
    if bl > 1e-6 and speed_sum > 0.02:
        lead = _add(target_pos, _scale(target_vel, params.naive_lead_time_s))
        lead_u = _unit_vec(_sub(lead, interceptor_pos))
        if _norm_vec(lead_u) > 1e-9:
            mixed = (
                (1.0 - bl) * los[0] + bl * lead_u[0],
                (1.0 - bl) * los[1] + bl * lead_u[1],
                (1.0 - bl) * los[2] + bl * lead_u[2],
            )
            mu = _unit_vec(mixed)
            return mu if _norm_vec(mu) > 1e-9 else los
    return los


def _solve_predictive(
    target_pos: Vec3,
    target_vel: Vec3,
    interceptor_pos: Vec3,
    dist: float,
    params: GuidanceParams,
) -> tuple[bool, float | None, Vec3 | None, Vec3 | None, float | None]:
    sp_plan = _command_speed(params, dist)
    solver_speed = sp_plan
    sol = compute_intercept(
        target_pos[0], target_pos[1], target_pos[2],
        target_vel[0], target_vel[1], target_vel[2],
        interceptor_pos[0], interceptor_pos[1], interceptor_pos[2],
        sp_plan,
    )
    if sol is not None:
        t_rough = sol[0]
        if params.t_hit_min_s <= t_rough <= params.t_hit_max_s:
            sp_actual = _command_speed(params, dist, t_go=t_rough)
            if abs(sp_actual - sp_plan) > 0.05:
                sol2 = compute_intercept(
                    target_pos[0], target_pos[1], target_pos[2],
                    target_vel[0], target_vel[1], target_vel[2],
                    interceptor_pos[0], interceptor_pos[1], interceptor_pos[2],
                    sp_actual,
                )
                if sol2 is not None:
                    sol = sol2
                    solver_speed = sp_actual
    if sol is None:
        return (False, None, None, None, None)
    t_hit, phx, phy, phz, ux, uy, uz = sol
    u_pred = (ux, uy, uz)
    if params.t_hit_min_s <= t_hit <= params.t_hit_max_s and _norm_vec(u_pred) > 1e-6:
        return (True, float(t_hit), (phx, phy, phz), u_pred, float(solver_speed))
    return (False, None, None, None, None)


def compute_guidance_command(gin: GuidanceInput) -> GuidanceCommand:
    params = gin.params
    t_pos0 = gin.target.position
    t_vel = gin.target.velocity
    if gin.target.measurement_delay_s > 0.0:
        t_pos = compensate_target_for_delay(
            t_pos0[0], t_pos0[1], t_pos0[2],
            t_vel[0], t_vel[1], t_vel[2],
            gin.target.measurement_delay_s,
        )
    else:
        t_pos = t_pos0
    i_pos = gin.interceptor.position
    i_vel = gin.interceptor.velocity
    aim_point = gin.aim_point_override if gin.aim_point_override is not None else t_pos
    dist = _norm_vec(_sub(t_pos, i_pos))

    sol_valid, t_hit, p_hit_raw, u_pred_raw, solver_speed = _solve_predictive(
        t_pos, t_vel, i_pos, dist, params,
    )
    p_hit_smooth: Vec3 | None = None
    u_pred: Vec3 | None = None
    if sol_valid and p_hit_raw is not None and u_pred_raw is not None:
        p_hit_smooth = smooth_intercept_point(
            gin.memory.smoothed_intercept_point,
            p_hit_raw,
            params.intercept_smoothing_alpha,
        )
        u_smooth = _unit_vec(_sub(p_hit_smooth, i_pos))
        u_pred = u_smooth if _norm_vec(u_smooth) > 1e-6 else u_pred_raw

    mode, valid_count, invalid_count, blend, transition = update_guidance_mode(
        gin.memory, params, sol_valid,
    )
    blend = _terminal_adjusted_blend(params, dist, blend)

    debug_active = False
    pn_active = False
    vc_log: float | None = None
    t_go_eff: float | None = None
    pursuit = _pursuit_unit(t_pos, t_vel, aim_point, i_pos, params)
    debug_required_speed: float | None = None
    if gin.debug_predictive_override and sol_valid and t_hit is not None and p_hit_raw is not None:
        u_dbg = _unit_vec(_sub(p_hit_raw, i_pos))
        if _norm_vec(u_dbg) > 1e-9:
            debug_required_speed = _compute_required_speed(i_pos, p_hit_raw, t_hit)
            if math.isfinite(debug_required_speed) and debug_required_speed < float("inf"):
                u_cmd = u_dbg
                speed_cmd = min(
                    params.interceptor_max_speed_m_s,
                    max(0.0, debug_required_speed + params.debug_speed_margin_m_s),
                )
                mode_out = "predict_dbg"
                debug_active = True
            else:
                u_cmd = (0.0, 0.0, 0.0)
                speed_cmd = 0.0
                mode_out = mode
        else:
            u_cmd = (0.0, 0.0, 0.0)
            speed_cmd = 0.0
            mode_out = mode
    else:
        u_pred_eff = u_pred
        blend_eff = blend
        if not sol_valid and blend_eff > 1e-6:
            flt = gin.memory.smoothed_intercept_point
            if flt is not None:
                u_flt = _unit_vec(_sub(flt, i_pos))
                if _norm_vec(u_flt) > 1e-6:
                    u_pred_eff = u_flt
                else:
                    blend_eff = 0.0
            else:
                blend_eff = 0.0
        if blend_eff > 1e-6 and u_pred_eff is not None:
            u_mix = (
                (1.0 - blend_eff) * pursuit[0] + blend_eff * u_pred_eff[0],
                (1.0 - blend_eff) * pursuit[1] + blend_eff * u_pred_eff[1],
                (1.0 - blend_eff) * pursuit[2] + blend_eff * u_pred_eff[2],
            )
            u_cmd = _unit_vec(u_mix)
            if _norm_vec(u_cmd) < 1e-9:
                u_cmd = pursuit
        else:
            u_cmd = pursuit
        mode_out = mode
        if not sol_valid:
            t_hit = None

        if params.use_pn_inspired and params.pn_blend_gain > 1e-9 and dist > 1e-6:
            steer, vc, pn_ok = pn_inspired_steering_vector(
                _sub(aim_point, i_pos),
                _sub(t_vel, i_vel),
                params.pn_navigation_constant,
                params.pn_min_closing_speed_m_s,
            )
            vc_log = vc
            if pn_ok:
                pn_w = effective_pn_inspired_blend(params, dist)
                u_pn = _unit_vec(_add(u_cmd, _scale(steer, pn_w)))
                if _norm_vec(u_pn) > 1e-9:
                    u_cmd = u_pn
                    pn_active = True
        else:
            pn_active = False
            vc_log = None

        if params.align_speed_to_solver and sol_valid and t_hit is not None and p_hit_raw is not None:
            t_go_raw = float(t_hit)
            prev_tgo = gin.memory.filtered_t_go_s
            t_go_eff = t_go_raw if prev_tgo is None else (
                params.t_go_filter_alpha * t_go_raw + (1.0 - params.t_go_filter_alpha) * prev_tgo
            )
            if prev_tgo is not None and params.t_go_filter_max_step_s > 1e-12:
                dc = t_go_eff - float(prev_tgo)
                if abs(dc) > params.t_go_filter_max_step_s:
                    t_go_eff = float(prev_tgo) + math.copysign(params.t_go_filter_max_step_s, dc)
            range_point = p_hit_smooth if params.align_speed_use_smooth_hit_range and p_hit_smooth is not None else p_hit_raw
            range_to_phit = _norm_vec(_sub(range_point, i_pos))
            speed_cmd = max(
                params.speed_vmin_m_s,
                min(params.max_speed_m_s, range_to_phit / max(t_go_eff, params.speed_tgo_min_s)),
            )
        else:
            t_go_eff = None
            speed_cmd = _command_speed(params, dist, t_go=t_hit if sol_valid else None)

    if debug_active:
        pn_active = False
        vc_log = None
        t_go_eff = None
        pursuit = _pursuit_unit(t_pos, t_vel, aim_point, i_pos, params)
    elif not params.align_speed_to_solver or not sol_valid:
        # Fallback path deliberately clears stale t_go state, matching live legacy behavior.
        if not sol_valid:
            p_hit_smooth = None
        if not sol_valid or not params.align_speed_to_solver:
            t_go_eff = None

    if gin.memory.previous_unit_command is not None and params.guidance_u_max_step_rad < math.pi - 1e-9:
        u_cmd = rotate_dir_toward(gin.memory.previous_unit_command, u_cmd, params.guidance_u_max_step_rad)

    v_cmd = _clamp_speed(params, _scale(u_cmd, speed_cmd))
    speed_actual = _norm_vec(v_cmd)
    if speed_actual > 1e-9:
        u_actual = (v_cmd[0] / speed_actual, v_cmd[1] / speed_actual, v_cmd[2] / speed_actual)
    else:
        u_actual = u_cmd

    next_t_go = t_go_eff if (params.align_speed_to_solver and sol_valid and not debug_active) else None
    memory_next = GuidanceMemory(
        mode=mode,
        predict_valid_count=valid_count,
        predict_invalid_count=invalid_count,
        filtered_t_go_s=next_t_go,
        smoothed_intercept_point=p_hit_smooth if sol_valid else gin.memory.smoothed_intercept_point,
        previous_unit_command=u_actual,
    )
    if debug_active:
        # Debug override keeps the mode/streak effects but does not seed t_go smoothing.
        memory_next = GuidanceMemory(
            mode=memory_next.mode,
            predict_valid_count=memory_next.predict_valid_count,
            predict_invalid_count=memory_next.predict_invalid_count,
            filtered_t_go_s=None,
            smoothed_intercept_point=memory_next.smoothed_intercept_point,
            previous_unit_command=memory_next.previous_unit_command,
        )

    pursuit_unit = _pursuit_unit(t_pos, t_vel, aim_point, i_pos, params)
    return GuidanceCommand(
        velocity_cmd=v_cmd,
        unit_cmd=u_actual,
        speed_cmd_m_s=speed_actual,
        mode=mode_out,
        solution_valid=sol_valid,
        t_go_raw_s=t_hit if sol_valid else None,
        t_go_effective_s=t_go_eff if sol_valid else None,
        intercept_point_raw=p_hit_raw if sol_valid else None,
        intercept_point_smoothed=p_hit_smooth if sol_valid else None,
        solver_speed_m_s=solver_speed if sol_valid else None,
        pursuit_unit=pursuit_unit,
        predictive_unit=u_pred if sol_valid else None,
        pn_inspired_active=pn_active,
        closing_speed_m_s=vc_log,
        memory_next=memory_next,
        mode_transition=transition,
        debug_predictive_override_active=debug_active,
        debug_required_speed_m_s=debug_required_speed,
    )


__all__ = [
    "Vec3",
    "TargetKinematics",
    "InterceptorKinematics",
    "GuidanceParams",
    "GuidanceMemory",
    "GuidanceInput",
    "GuidanceCommand",
    "compute_guidance_command",
    "effective_pn_inspired_blend",
    "pn_inspired_steering_vector",
    "rotate_dir_toward",
    "update_guidance_mode",
]
