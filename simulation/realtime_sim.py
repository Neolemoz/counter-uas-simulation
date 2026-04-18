#!/usr/bin/env python3
"""Offline timestep engine: naive, predictive, and PN guidance; turn-rate limited."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Literal

import matplotlib.pyplot as plt
import numpy as np

_SIM_ROOT = Path(__file__).resolve().parent
if str(_SIM_ROOT) not in sys.path:
    sys.path.insert(0, str(_SIM_ROOT))

from core.dynamics import update_velocity, update_velocity_from_pn_lateral
from defense_types import (
    InterceptorSpec,
    TargetParams,
    feasibility_cv,
    time_to_intercept_cv,
)
from guidance.naive import compute_naive_direction
from guidance.pn import compute_pn_acceleration
from guidance.predictive import compute_predictive_direction

ManeuverAxis = Literal["y", "z"]

# Pure PN navigation constant (—)
PN_GAIN = 3.0

# Maneuvering target: p(t) = p0 + v_lin*t + A*sin(omega*t)*e_axis (X from v_lin.x, axis = Y or Z weave)
TARGET_MANEUVER_A_M = 10.0
TARGET_MANEUVER_OMEGA_RAD_S = 0.55
TARGET_MANEUVER_AXIS: ManeuverAxis = "y"

# Per-method intercept evaluation (closest approach)
INTERCEPT_EVAL_THRESHOLD_M = 1.0


def evaluate_intercept(
    target: np.ndarray,
    interceptor: np.ndarray,
    dt: float,
    threshold_m: float,
) -> tuple[float, float, bool]:
    """d[k] = ||p_i[k]-p_t[k]||; miss = min d; t* = argmin_k (k*dt); success if miss < threshold."""
    diff = interceptor - target
    dist = np.linalg.norm(diff, axis=1)
    i_min = int(np.argmin(dist))
    miss_m = float(dist[i_min])
    t_min = float(i_min) * float(dt)
    success = miss_m < float(threshold_m)
    return miss_m, t_min, success


def target_position_velocity(
    p0: np.ndarray,
    v_lin: np.ndarray,
    t: float,
    A: float,
    omega: float,
    axis: ManeuverAxis,
) -> tuple[np.ndarray, np.ndarray]:
    """p = p0 + v_lin*t + A*sin(omega*t) on Y or Z; v = dp/dt (analytic)."""
    w = np.zeros(3, dtype=float)
    if axis == "y":
        w[1] = 1.0
    else:
        w[2] = 1.0
    osc = float(A) * np.sin(float(omega) * float(t))
    p = p0 + v_lin * float(t) + w * osc
    v = v_lin + w * (float(A) * float(omega) * np.cos(float(omega) * float(t)))
    return p.astype(float), v.astype(float)


def desired_unit_direction(
    p_t: np.ndarray,
    v_t: np.ndarray,
    p_i: np.ndarray,
    v_i: np.ndarray,
    s_i: float,
    dt: float,
    mode: Literal["naive", "predictive"],
) -> np.ndarray:
    """Unit direction for ``update_velocity`` (naive / predictive only; PN uses lateral accel path in ``run_sim``)."""
    if mode == "naive":
        return compute_naive_direction(p_t, p_i)
    u = compute_predictive_direction(p_t, v_t, p_i, s_i)
    if u is None:
        return compute_naive_direction(p_t, p_i)
    return u


def _replanning_best_by_tti(
    p_t: np.ndarray,
    v_t: np.ndarray,
    candidates: list[InterceptorSpec],
) -> tuple[int, float] | None:
    """STEP 7 diagnostic: among all candidates with CV feasible intercept, return (index, min TTI)."""
    eligible: list[tuple[int, float]] = []
    for idx, spec in enumerate(candidates):
        if not feasibility_cv(p_t, v_t, spec.position0, spec.speed):
            continue
        tti = time_to_intercept_cv(p_t, v_t, spec.position0, spec.speed)
        if tti is None:
            continue
        eligible.append((idx, float(tti)))
    if not eligible:
        return None
    return min(eligible, key=lambda r: r[1])


def run_selected_interceptor_closed_loop(
    *,
    p0: np.ndarray,
    v_lin: np.ndarray,
    A_m: float,
    omega: float,
    m_axis: ManeuverAxis,
    best_spec: InterceptorSpec,
    interceptor_candidates: list[InterceptorSpec],
    committed_interceptor_idx: int,
    dt: float,
    max_time: float,
    max_turn_rate: float,
    range_threshold_m: float,
    replan_interval_s: float = 0.5,
) -> tuple[np.ndarray, np.ndarray, float, bool]:
    """STEP 6–7: single committed asset; predictive + PN loop; periodic CV replan diagnostics (no switch)."""
    s_i = float(best_spec.speed)
    p_t, v_t = target_position_velocity(p0, v_lin, 0.0, A_m, omega, m_axis)
    p_sel = np.asarray(best_spec.position0, dtype=float).copy()
    v_zero = np.zeros(3, dtype=float)
    u0 = desired_unit_direction(p_t, v_t, p_sel, v_zero, s_i, dt, "predictive")
    v_sel = u0 * s_i

    tgt_hist: list[np.ndarray] = [p_t.copy()]
    sel_hist: list[np.ndarray] = [p_sel.copy()]

    t = 0.0
    hit = False
    next_replan_at = float(replan_interval_s)
    while t < max_time - 1e-12:
        t += dt
        p_t, v_t = target_position_velocity(p0, v_lin, t, A_m, omega, m_axis)
        _ = compute_predictive_direction(p_t, v_t, p_sel, s_i)
        a_pn = compute_pn_acceleration(p_t, v_t, p_sel, v_sel, PN_GAIN)
        v_sel = update_velocity_from_pn_lateral(v_sel, a_pn, s_i, max_turn_rate, dt)
        p_sel = p_sel + v_sel * dt
        tgt_hist.append(p_t.copy())
        sel_hist.append(p_sel.copy())
        while t + 1e-9 >= next_replan_at:
            t_log = next_replan_at
            best_now = _replanning_best_by_tti(p_t, v_t, interceptor_candidates)
            if best_now is None:
                print(f"[Replan t={t_log:.2f}s] Best: none (committed: I{committed_interceptor_idx})")
            else:
                bi, tti = best_now
                print(f"[Replan t={t_log:.2f}s] Best: I{bi}, TTI={tti:.2f} (committed: I{committed_interceptor_idx})")
            next_replan_at += float(replan_interval_s)
        if float(np.linalg.norm(p_sel - p_t)) < float(range_threshold_m):
            hit = True
            break

    return np.stack(tgt_hist), np.stack(sel_hist), t, hit


def run_sim() -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    bool,
    float,
    float,
    tuple[int, InterceptorSpec, float] | None,
    list[InterceptorSpec],
]:
    dt = 0.05
    max_time = 20.0
    r_int = 1.0
    max_turn_rate = np.deg2rad(10.0)
    s_i = 18.0

    p0 = np.array([0.0, 0.0, 20.0], dtype=float)
    # Forward in X; weave on TARGET_MANEUVER_AXIS via sine; linear Y set 0 so maneuver is visible on that axis
    v_lin = np.array([8.0, 0.0, -1.0], dtype=float)
    A_m = TARGET_MANEUVER_A_M
    omega = TARGET_MANEUVER_OMEGA_RAD_S
    m_axis = TARGET_MANEUVER_AXIS

    p_t, v_t = target_position_velocity(p0, v_lin, 0.0, A_m, omega, m_axis)
    # Distinct launch sites (STEP 3): I0 naive, I1 predictive, I2 PN — same speed, different geometry
    p_n = np.array([-40.0, -20.0, 0.0], dtype=float)
    p_p = np.array([40.0, -20.0, 0.0], dtype=float)
    p_k = np.array([0.0, 40.0, 0.0], dtype=float)

    target_params = TargetParams(
        p0=p0.copy(),
        v_lin=v_lin.copy(),
        A=A_m,
        omega=omega,
        axis=m_axis,
    )
    interceptor_candidates = [
        InterceptorSpec("naive", p_n.copy(), s_i, "naive"),
        InterceptorSpec("predictive", p_p.copy(), s_i, "predictive"),
        InterceptorSpec("pn", p_k.copy(), s_i, "pn"),
    ]
    print("\n=== Interceptor Feasibility (t = 0) ===\n")
    print(f"{'ID':<6}{'Feasible':<12}{'TTI (s)'}")
    print("--------------------------------")
    candidate_eval: list[tuple[int, InterceptorSpec, bool, float | None]] = []
    for idx, spec in enumerate(interceptor_candidates):
        feasible = feasibility_cv(p_t, v_t, spec.position0, spec.speed)
        t_hit = time_to_intercept_cv(p_t, v_t, spec.position0, spec.speed)
        candidate_eval.append((idx, spec, feasible, t_hit))
        tti_str = f"{t_hit:.2f}" if t_hit is not None else "None"
        print(f"I{idx}    {str(feasible):<12}{tti_str}")
    print()

    # STEP 5: select best by minimum TTI among feasible; PN excluded from pool (predictive/naive baseline only).
    eligible = [
        (idx, spec, t_hit)
        for idx, spec, feasible, t_hit in candidate_eval
        if feasible and t_hit is not None and spec.guidance != "pn"
    ]
    print("=== Best Interceptor Selection (t = 0) ===")
    print("Selection pool: Predictive / Naive only")
    print("Excluded from selection: PN")
    print()
    best_selection: tuple[int, InterceptorSpec, float] | None
    if not eligible:
        best_selection = None
        print("Selected ID: (none)")
        print("TTI: N/A")
        print("Position: N/A")
    else:
        best_idx, best_spec, best_t_hit = min(eligible, key=lambda r: r[2])
        best_selection = (best_idx, best_spec, best_t_hit)
        print(f"Selected ID: I{best_idx}")
        print(f"TTI: {best_t_hit:.2f}")
        print(f"Position: {best_spec.position0}")
    print()

    print("Interceptor positions:")
    for idx, spec in enumerate(interceptor_candidates):
        print(f"I{idx}: {spec.position0}")
    print("Target state:")
    print("p_t:", p_t)
    print("v_t:", v_t)
    for idx, spec in enumerate(interceptor_candidates):
        d = float(np.linalg.norm(p_t - spec.position0))
        print(f"I{idx} distance: {d}")

    v_zero = np.zeros(3, dtype=float)
    # Per-site initial velocity (speed s_i); each branch must use its own launch position, not p_n only.
    u_n = desired_unit_direction(p_t, v_t, p_n, v_zero, s_i, dt, "naive")
    v_n = u_n * s_i

    u_p = desired_unit_direction(p_t, v_t, p_p, v_zero, s_i, dt, "predictive")
    v_p = u_p * s_i

    # PN: same bootstrap contract as predictive — CV lead at p_k if available, else naive LOS (inside desired_unit_direction).
    u_k = desired_unit_direction(p_t, v_t, p_k, v_zero, s_i, dt, "predictive")
    v_k = u_k * s_i

    tgt_hist: list[np.ndarray] = [p_t.copy()]
    naive_hist: list[np.ndarray] = [p_n.copy()]
    pred_hist: list[np.ndarray] = [p_p.copy()]
    pn_hist: list[np.ndarray] = [p_k.copy()]

    t = 0.0
    success = False

    while t < max_time - 1e-12:
        t += dt
        p_t, v_t = target_position_velocity(p0, v_lin, t, A_m, omega, m_axis)

        u_naive = desired_unit_direction(p_t, v_t, p_n, v_n, s_i, dt, "naive")
        v_n = update_velocity(v_n, u_naive, s_i, max_turn_rate, dt)
        p_n = p_n + v_n * dt

        u_pred = desired_unit_direction(p_t, v_t, p_p, v_p, s_i, dt, "predictive")
        v_p = update_velocity(v_p, u_pred, s_i, max_turn_rate, dt)
        p_p = p_p + v_p * dt

        # PN: commanded accel from guidance.pn → lateral component in velocity frame → Euler step
        # → direction at constant |v| is implicit in update_velocity_from_pn_lateral → same turn-rate cap as others
        a_pn = compute_pn_acceleration(p_t, v_t, p_k, v_k, PN_GAIN)
        v_k = update_velocity_from_pn_lateral(v_k, a_pn, s_i, max_turn_rate, dt)
        p_k = p_k + v_k * dt

        tgt_hist.append(p_t.copy())
        naive_hist.append(p_n.copy())
        pred_hist.append(p_p.copy())
        pn_hist.append(p_k.copy())

        d_n = float(np.linalg.norm(p_n - p_t))
        d_p = float(np.linalg.norm(p_p - p_t))
        if d_n < r_int and d_p < r_int:
            success = True
            break

    T = np.stack(tgt_hist)
    N = np.stack(naive_hist)
    P = np.stack(pred_hist)
    K = np.stack(pn_hist)

    return T, N, P, K, success, t, dt, best_selection, interceptor_candidates


def _distance_profile(target: np.ndarray, interceptor: np.ndarray) -> np.ndarray:
    """d(t) = ||p_interceptor - p_target|| at each saved sample."""
    return np.linalg.norm(interceptor - target, axis=1)


def print_interception_results(
    rows: list[tuple[str, float, float, bool]],
    threshold_m: float,
) -> None:
    """Block print + summary table (miss = min d(t), time = argmin * dt)."""
    th = float(threshold_m)
    print("\n=== Interception Results ===\n")
    for title, miss, tmin, ok in rows:
        print(f"[{title}]")
        print(f"Miss Distance: {miss:.3f} m")
        print(f"Intercept Time: {tmin:.2f} s")
        print(f"Success: {ok}\n")

    print(f"(Success if miss distance < {th:.1f} m)\n")
    print(f"{'Method':<12} {'Miss(m)':>10} {'Time(s)':>10} {'Success':>10}")
    print("-" * 44)
    for title, miss, tmin, ok in rows:
        print(f"{title:<12} {miss:>10.3f} {tmin:>10.2f} {str(ok):>10}")
    print("-" * 44)


def plot_distance_vs_target(
    T: np.ndarray,
    series: list[tuple[str, np.ndarray, str]],
    dt: float,
    threshold_m: float,
) -> None:
    """Display d(t) vs time for each guidance trace."""
    t_axis = np.arange(T.shape[0], dtype=float) * float(dt)
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for label, traj, color in series:
        d = _distance_profile(T, traj)
        ax.plot(t_axis, d, label=label, color=color, linewidth=2.0)
    ax.axhline(
        float(threshold_m),
        color="0.4",
        linestyle="--",
        linewidth=1.0,
        label=f"{threshold_m:.1f} m threshold",
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance to target (m)")
    ax.set_title("Interception: range vs time")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()


def main() -> None:
    T, N, P, K, success, t_elapsed, dt, best_selection, interceptor_candidates = run_sim()

    th = INTERCEPT_EVAL_THRESHOLD_M
    ev_n = evaluate_intercept(T, N, dt, th)
    ev_p = evaluate_intercept(T, P, dt, th)
    ev_k = evaluate_intercept(T, K, dt, th)

    print_interception_results(
        [
            ("Naive", ev_n[0], ev_n[1], ev_n[2]),
            ("Predictive", ev_p[0], ev_p[1], ev_p[2]),
            ("PN", ev_k[0], ev_k[1], ev_k[2]),
        ],
        th,
    )

    print(
        "\nDual stop (naive+pred loop criterion):",
        "SUCCESS" if success else "FAIL",
        f"  wall time: {t_elapsed:.3f} s",
    )

    # STEP 6–7: closed-loop selected asset; STEP 7 logs hypothetical best-by-TTI (no reassignment).
    if best_selection is not None:
        sel_idx, sel_spec, _ = best_selection
        p0_s = np.array([0.0, 0.0, 20.0], dtype=float)
        v_lin_s = np.array([8.0, 0.0, -1.0], dtype=float)
        A_s = TARGET_MANEUVER_A_M
        om_s = TARGET_MANEUVER_OMEGA_RAD_S
        ax_s = TARGET_MANEUVER_AXIS
        dt_s = 0.05
        max_t_s = 20.0
        max_turn_s = np.deg2rad(10.0)
        r_hit = 1.0
        print("\n=== Selected Interceptor Run ===")
        print(f"Selected ID: I{sel_idx}")
        T_sel, S_sel, _t_sel, _ = run_selected_interceptor_closed_loop(
            p0=p0_s,
            v_lin=v_lin_s,
            A_m=A_s,
            omega=om_s,
            m_axis=ax_s,
            best_spec=sel_spec,
            interceptor_candidates=interceptor_candidates,
            committed_interceptor_idx=sel_idx,
            dt=dt_s,
            max_time=max_t_s,
            max_turn_rate=max_turn_s,
            range_threshold_m=r_hit,
            replan_interval_s=0.5,
        )
        ev_sel = evaluate_intercept(T_sel, S_sel, dt_s, th)
        print(f"Miss Distance: {ev_sel[0]:.3f} m")
        print(f"Intercept Time: {ev_sel[1]:.2f} s")
        print(f"Success: {ev_sel[2]}")
        fig_sel = plt.figure(figsize=(8, 6))
        axs = fig_sel.add_subplot(111, projection="3d")
        axs.plot(T_sel[:, 0], T_sel[:, 1], T_sel[:, 2], color="C0", label="Target", linewidth=2)
        axs.plot(S_sel[:, 0], S_sel[:, 1], S_sel[:, 2], color="C4", label="Selected interceptor", linewidth=2)
        axs.set_xlabel("X")
        axs.set_ylabel("Y")
        axs.set_zlabel("Z")
        axs.set_title("STEP 6: selected interceptor (predictive + PN)")
        axs.legend(loc="upper left", fontsize=9)
        plt.tight_layout()

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(T[:, 0], T[:, 1], T[:, 2], color="C0", label="Target", linewidth=2)
    ax.plot(N[:, 0], N[:, 1], N[:, 2], color="C2", label="Naive", linewidth=2)
    ax.plot(P[:, 0], P[:, 1], P[:, 2], color="C1", label="Predictive", linewidth=2)
    ax.plot(K[:, 0], K[:, 1], K[:, 2], color="C3", label="PN", linewidth=2)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Interceptor trajectories (3D)")
    ax.legend(loc="upper left", fontsize=9)
    plt.tight_layout()

    plot_distance_vs_target(
        T,
        [
            ("Naive", N, "C2"),
            ("Predictive", P, "C1"),
            ("PN", K, "C3"),
        ],
        dt,
        th,
    )

    plt.show()


if __name__ == "__main__":
    main()
