from __future__ import annotations

from typing import Optional

import numpy as np

try:
    # Repo code commonly runs with "simulation/" on PYTHONPATH so "core" resolves.
    from core.utils import normalize
except ModuleNotFoundError:  # allows running this file directly
    def normalize(vector: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(vector))
        if n < 1e-15:
            raise ValueError("Cannot normalize zero-length vector")
        return vector / n


def compute_tti_target(
    p_target: np.ndarray,
    v_target: np.ndarray,
    protected_center: np.ndarray,
    protected_radius: float,
) -> Optional[float]:
    """
    Time-to-impact of a target reaching the boundary of a protected sphere.

    Solve for the smallest t > 0 such that:
        || (p_target + v_target * t) - protected_center || = protected_radius

    Returns None if no positive solution exists, or if the target is moving away.
    """
    p = np.asarray(p_target, dtype=float).reshape(3)
    v = np.asarray(v_target, dtype=float).reshape(3)
    c = np.asarray(protected_center, dtype=float).reshape(3)
    R = float(protected_radius)
    if not np.isfinite(R) or R <= 0.0:
        return None

    r0 = p - c
    vv = float(np.dot(v, v))
    if vv <= 1e-15:
        return None

    rv = float(np.dot(r0, v))
    # "Moving away" gate (require approaching the center along current velocity).
    if rv >= 0.0:
        return None

    rr = float(np.dot(r0, r0))
    a = vv
    b = 2.0 * rv
    c0 = rr - R * R

    disc = b * b - 4.0 * a * c0
    if disc < 0.0:
        return None

    sqrt_d = float(np.sqrt(disc))
    t0 = (-b - sqrt_d) / (2.0 * a)
    t1 = (-b + sqrt_d) / (2.0 * a)
    candidates = [t for t in (t0, t1) if t > 0.0 and np.isfinite(t)]
    if not candidates:
        return None
    return float(min(candidates))


def is_target_detected_by_radar(
    p_target: np.ndarray,
    radar_position: np.ndarray,
    radar_range: float,
) -> bool:
    """Return True if ||p_target - radar_position|| <= radar_range."""
    p = np.asarray(p_target, dtype=float).reshape(3)
    r = np.asarray(radar_position, dtype=float).reshape(3)
    rr = float(radar_range)
    if not np.isfinite(rr) or rr < 0.0:
        return False
    d = p - r
    distance = float(np.linalg.norm(d))
    return bool(distance <= rr)


def is_tracking_ready(
    detection_time: float,
    current_time: float,
    tracking_delay: float,
) -> bool:
    """Return True iff current_time >= detection_time + tracking_delay."""
    dt = float(detection_time)
    ct = float(current_time)
    td = float(tracking_delay)
    return bool(ct >= (dt + td))


def compute_total_required_time(
    intercept_time: float,
    detection_latency: float,
    tracking_delay: float,
    launch_delay: float,
) -> float:
    """T_required = detection_latency + tracking_delay + launch_delay + intercept_time."""
    it = float(intercept_time)
    dl = float(detection_latency)
    td = float(tracking_delay)
    ld = float(launch_delay)
    if it < 0.0 or dl < 0.0 or td < 0.0 or ld < 0.0:
        raise ValueError("All time inputs must be non-negative.")
    return float(dl + td + ld + it)


def compute_total_required_time_extended(
    intercept_time: float,
    detection_latency: float,
    tracking_delay: float,
    launch_delay: float,
    processing_delay: float,
) -> float:
    """T_required = detection_latency + tracking_delay + launch_delay + processing_delay + intercept_time."""
    it = float(intercept_time)
    dl = float(detection_latency)
    td = float(tracking_delay)
    ld = float(launch_delay)
    pd = float(processing_delay)
    if it < 0.0 or dl < 0.0 or td < 0.0 or ld < 0.0 or pd < 0.0:
        raise ValueError("All time inputs must be non-negative.")
    return float(dl + td + ld + pd + it)


def evaluate_feasibility(
    T_required: float,
    TTI_target: Optional[float],
) -> dict:
    """
    Evaluate engagement feasibility against target time-to-impact (TTI_target).

    Logic:
      - If TTI_target is None: feasible=False, margin=None
      - Else: feasible = (T_required < TTI_target), margin = (TTI_target - T_required)
    """
    tr = float(T_required)
    if TTI_target is None:
        return {
            "feasible": False,
            "T_required": tr,
            "TTI_target": None,
            "margin": None,
        }
    tt = float(TTI_target)
    return {
        "feasible": bool(tr < tt),
        "T_required": tr,
        "TTI_target": tt,
        "margin": float(tt - tr),
    }


def compute_intercept_time(
    p_target: np.ndarray,
    v_target: np.ndarray,
    p_interceptor: np.ndarray,
    interceptor_speed: float,
) -> Optional[float]:
    """
    Thin wrapper for intercept-time computation (constant-speed interceptor, CV target).

    This does not modify the existing solver; it simply forwards to it so a higher-level
    pipeline can call a semantically named function.
    """
    return solve_intercept_time(
        np.asarray(p_target, dtype=float).reshape(3),
        np.asarray(v_target, dtype=float).reshape(3),
        np.asarray(p_interceptor, dtype=float).reshape(3),
        float(interceptor_speed),
    )


def compute_intercept_decision(
    p_target: np.ndarray,
    v_target: np.ndarray,
    p_interceptor: np.ndarray,
    interceptor_speed: float,
    protected_center: np.ndarray,
    protected_radius: float,
    detection_latency: float,
    tracking_delay: float,
    launch_delay: float,
) -> dict:
    """
    One-shot interception decision pipeline.

    Calls:
      - compute_intercept_time()
      - compute_tti_target()
      - compute_total_required_time()
      - evaluate_feasibility()
    """
    intercept_time = compute_intercept_time(p_target, v_target, p_interceptor, interceptor_speed)
    tti_target = compute_tti_target(p_target, v_target, protected_center, protected_radius)

    if intercept_time is None:
        return {
            "intercept_time": None,
            "TTI_target": float(tti_target) if tti_target is not None else None,
            "T_required": None,
            "margin": None,
            "feasible": False,
            "reason": "No intercept solution (intercept_time=None).",
        }

    try:
        t_required = compute_total_required_time(
            intercept_time=float(intercept_time),
            detection_latency=float(detection_latency),
            tracking_delay=float(tracking_delay),
            launch_delay=float(launch_delay),
        )
    except ValueError as e:
        return {
            "intercept_time": float(intercept_time),
            "TTI_target": float(tti_target) if tti_target is not None else None,
            "T_required": None,
            "margin": None,
            "feasible": False,
            "reason": f"Invalid delay inputs: {e}",
        }

    fe = evaluate_feasibility(T_required=t_required, TTI_target=tti_target)
    margin = fe["margin"]
    feasible = bool(fe["feasible"])

    if tti_target is None:
        reason = "TTI_target=None (no boundary hit / target moving away), so feasible=False."
    elif feasible:
        reason = f"T_required < TTI_target (margin={float(margin):.6g} s)."
    else:
        reason = f"T_required >= TTI_target (margin={float(margin):.6g} s), so too late."

    return {
        "intercept_time": float(intercept_time),
        "TTI_target": float(tti_target) if tti_target is not None else None,
        "T_required": float(t_required),
        "margin": float(margin) if margin is not None else None,
        "feasible": feasible,
        "reason": reason,
    }


def evaluate_intercept_feasibility_realistic(
    p_target: np.ndarray,
    v_target: np.ndarray,
    p_interceptor: np.ndarray,
    interceptor_speed: float,
    protected_center: np.ndarray,
    protected_radius: float,
    radar_position: np.ndarray,
    radar_range: float,
    detection_time: float,
    current_time: float,
    detection_latency: float,
    tracking_delay: float,
    launch_delay: float,
    processing_delay: float = 0.0,
) -> dict:
    """
    Realistic feasibility gate: radar detect + tracking readiness + delay model + TTI compare.

    Logic order:
      1) radar detection gate
      2) tracking readiness gate
      3) compute TTI_target and T_required
      4) feasible = (T_required < TTI_target)
    """
    detected = is_target_detected_by_radar(p_target, radar_position, radar_range)
    tracking_ready = is_tracking_ready(detection_time, current_time, tracking_delay)

    if not detected:
        return {
            "detected": False,
            "tracking_ready": bool(tracking_ready),
            "T_required": None,
            "TTI_target": None,
            "feasible": False,
            "reason": "not detected",
        }

    if not tracking_ready:
        return {
            "detected": True,
            "tracking_ready": False,
            "T_required": None,
            "TTI_target": None,
            "feasible": False,
            "reason": "tracking not ready",
        }

    intercept_time = compute_intercept_time(p_target, v_target, p_interceptor, interceptor_speed)
    if intercept_time is None:
        return {
            "detected": True,
            "tracking_ready": True,
            "T_required": None,
            "TTI_target": float(compute_tti_target(p_target, v_target, protected_center, protected_radius))
            if compute_tti_target(p_target, v_target, protected_center, protected_radius) is not None
            else None,
            "feasible": False,
            "reason": "no intercept solution",
        }

    try:
        t_required = compute_total_required_time_extended(
            intercept_time=float(intercept_time),
            detection_latency=float(detection_latency),
            tracking_delay=float(tracking_delay),
            launch_delay=float(launch_delay),
            processing_delay=float(processing_delay),
        )
    except ValueError as e:
        return {
            "detected": True,
            "tracking_ready": True,
            "T_required": None,
            "TTI_target": None,
            "feasible": False,
            "reason": f"invalid delays: {e}",
        }

    tti_target = compute_tti_target(p_target, v_target, protected_center, protected_radius)
    fe = evaluate_feasibility(T_required=t_required, TTI_target=tti_target)
    feasible = bool(fe["feasible"])
    margin = fe["margin"]

    if tti_target is None:
        reason = "TTI_target is None"
    elif feasible:
        reason = f"feasible (margin={float(margin):.6g} s)"
    else:
        reason = f"not feasible (margin={float(margin):.6g} s)"

    return {
        "detected": True,
        "tracking_ready": True,
        "T_required": float(t_required),
        "TTI_target": float(tti_target) if tti_target is not None else None,
        "feasible": feasible,
        "reason": reason,
    }


def compute_intercept_decision_realistic(
    p_target: np.ndarray,
    v_target: np.ndarray,
    p_interceptor: np.ndarray,
    interceptor_speed: float,
    protected_center: np.ndarray,
    protected_radius: float,
    radar_position: np.ndarray,
    radar_range: float,
    detection_time: float,
    current_time: float,
    detection_latency: float,
    tracking_delay: float,
    launch_delay: float,
    processing_delay: float = 0.0,
) -> dict:
    """
    Full realistic decision pipeline (gates + delays + intercept math).

    Flow:
      1) detection check
      2) tracking readiness
      3) compute intercept_time
      4) compute TTI_target
      5) compute T_required
      6) evaluate feasibility
    """
    detected = is_target_detected_by_radar(p_target, radar_position, radar_range)
    tracking_ready = is_tracking_ready(detection_time, current_time, tracking_delay)

    if not detected:
        return {
            "detected": False,
            "tracking_ready": bool(tracking_ready),
            "intercept_time": None,
            "T_required": None,
            "TTI_target": None,
            "margin": None,
            "feasible": False,
            "reason": "not detected",
        }

    if not tracking_ready:
        return {
            "detected": True,
            "tracking_ready": False,
            "intercept_time": None,
            "T_required": None,
            "TTI_target": None,
            "margin": None,
            "feasible": False,
            "reason": "tracking not ready",
        }

    intercept_time = compute_intercept_time(p_target, v_target, p_interceptor, interceptor_speed)
    if intercept_time is None:
        return {
            "detected": True,
            "tracking_ready": True,
            "intercept_time": None,
            "T_required": None,
            "TTI_target": None,
            "margin": None,
            "feasible": False,
            "reason": "no intercept solution",
        }

    tti_target = compute_tti_target(p_target, v_target, protected_center, protected_radius)
    try:
        t_required = compute_total_required_time_extended(
            intercept_time=float(intercept_time),
            detection_latency=float(detection_latency),
            tracking_delay=float(tracking_delay),
            launch_delay=float(launch_delay),
            processing_delay=float(processing_delay),
        )
    except ValueError as e:
        return {
            "detected": True,
            "tracking_ready": True,
            "intercept_time": float(intercept_time),
            "T_required": None,
            "TTI_target": float(tti_target) if tti_target is not None else None,
            "margin": None,
            "feasible": False,
            "reason": f"invalid delays: {e}",
        }

    fe = evaluate_feasibility(T_required=t_required, TTI_target=tti_target)
    feasible = bool(fe["feasible"])
    margin = fe["margin"]

    if tti_target is None:
        reason = "TTI_target is None"
    elif feasible:
        reason = f"T_required < TTI_target (margin={float(margin):.6g} s)"
    else:
        reason = f"T_required >= TTI_target (margin={float(margin):.6g} s)"

    return {
        "detected": True,
        "tracking_ready": True,
        "intercept_time": float(intercept_time),
        "T_required": float(t_required),
        "TTI_target": float(tti_target) if tti_target is not None else None,
        "margin": float(margin) if margin is not None else None,
        "feasible": feasible,
        "reason": reason,
    }


def solve_intercept_time(
    p_t0: np.ndarray,
    v_t: np.ndarray,
    p_i0: np.ndarray,
    s_i: float,
) -> float | None:
    r0 = p_t0 - p_i0
    vv = float(np.dot(v_t, v_t))
    rv = float(np.dot(r0, v_t))
    rr = float(np.dot(r0, r0))

    a = vv - s_i**2
    b = 2.0 * rv
    c = rr

    eps = 1e-12
    candidates: list[float] = []

    if abs(a) < eps:
        if abs(b) < eps:
            return None
        t = -c / b
        if t > 0:
            candidates.append(t)
    else:
        disc = b * b - 4.0 * a * c
        if disc < 0:
            return None
        sqrt_d = np.sqrt(disc)
        for t in ((-b - sqrt_d) / (2.0 * a), (-b + sqrt_d) / (2.0 * a)):
            if t > 0:
                candidates.append(float(t))

    if not candidates:
        return None

    valid: list[float] = []
    for t in candidates:
        d = (p_t0 + v_t * t) - p_i0
        lhs = float(np.linalg.norm(d))
        rhs = s_i * t
        if abs(lhs - rhs) <= 1e-6 * max(1.0, lhs, rhs):
            valid.append(t)

    if not valid:
        return None
    return min(valid)


def compute_intercept(
    p_t0: np.ndarray,
    v_t: np.ndarray,
    p_i0: np.ndarray,
    s_i: float,
) -> tuple[float, np.ndarray, np.ndarray] | None:
    t = solve_intercept_time(p_t0, v_t, p_i0, s_i)
    if t is None:
        return None

    p_hit = p_t0 + v_t * t
    delta = p_hit - p_i0
    u = normalize(delta)
    return t, p_hit, u


if __name__ == "__main__":
    # Simple sanity check: target at x=10 moving toward origin at 2 m/s,
    # protected sphere radius=4 -> boundary hit at t=(10-4)/2 = 3 s.
    p = np.array([10.0, 0.0, 0.0])
    v = np.array([-2.0, 0.0, 0.0])
    center = np.array([0.0, 0.0, 0.0])
    tti = compute_tti_target(p, v, center, protected_radius=4.0)
    print(f"compute_tti_target example: tti={tti} (expected ~3.0)")

    # Radar detection gate tests.
    p1 = np.array([300.0, 0.0, 0.0])
    rp = np.array([0.0, 0.0, 0.0])
    print(
        "radar case 1:",
        is_target_detected_by_radar(p1, rp, radar_range=500.0),
        "(expected True)",
    )
    p2 = np.array([700.0, 0.0, 0.0])
    print(
        "radar case 2:",
        is_target_detected_by_radar(p2, rp, radar_range=500.0),
        "(expected False)",
    )

    # Tracking readiness gate tests.
    print("tracking ready case 1:", is_tracking_ready(5.0, 6.9, 2.0), "(expected False)")
    print("tracking ready case 2:", is_tracking_ready(5.0, 7.0, 2.0), "(expected True)")

    # Realistic feasibility gate smoke checks (early-exit reasons).
    p_t = np.array([700.0, 0.0, 0.0])
    v_t = np.array([-20.0, 0.0, 0.0])
    p_i = np.array([0.0, 0.0, 0.0])
    c0 = np.array([0.0, 0.0, 0.0])
    print(
        "realistic gate (not detected):",
        evaluate_intercept_feasibility_realistic(
            p_t, v_t, p_i, 35.0, c0, 50.0, c0, 500.0,
            detection_time=0.0, current_time=10.0,
            detection_latency=1.0, tracking_delay=2.0, launch_delay=1.0, processing_delay=0.0,
        )["reason"],
    )
