#!/usr/bin/env python3
"""Offline timestep engine: naive, predictive, and PN guidance; turn-rate limited."""

from __future__ import annotations

import sys
from dataclasses import dataclass
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

# Pure PN navigation constant (—); STEP A3: modest gain increase with higher PN accel cap
PN_GAIN = 4.0

# Maneuvering target: p(t) = p0 + v_lin*t + A*sin(omega*t)*e_axis (X from v_lin.x, axis = Y or Z weave)
TARGET_MANEUVER_A_M = 10.0
TARGET_MANEUVER_OMEGA_RAD_S = 0.55
TARGET_MANEUVER_AXIS: ManeuverAxis = "y"

# Per-method intercept evaluation (closest approach)
INTERCEPT_EVAL_THRESHOLD_M = 1.0

# Observed target position noise (Gaussian on position only; true trajectory unchanged for kinematics / eval)
TARGET_POSITION_NOISE_STD_M = 0.5
TARGET_POSITION_NOISE_RNG_SEED = 42

# Dynamic handoff during selected run: switch if new CV TTI is better by at least this margin (seconds) — STEP A3
SWITCH_TTI_MARGIN_S = 0.35
# Single-switch policy: handoff only before this time (seconds); at most one switch per run — STEP A3
SWITCH_ALLOW_UNTIL_S = 2.0

# STEP A3: higher turn-rate cap so PN can use the raised accel limit under stricter TTI switch rule
MAX_TURN_RATE_DEG_S = 16.0

# Monte Carlo (STEP A1): per-trial RNG stream offset so each trial gets independent position noise draws
MC_TRIAL_SEED_BASE = 10_000
# STEP A2: randomized target initial conditions per MC trial (maneuver + noise models unchanged)
MC_IC_POS_X = (-10.0, 10.0)
MC_IC_POS_Y = (-10.0, 10.0)
MC_IC_POS_Z = (15.0, 25.0)
MC_IC_SPEED = (6.0, 12.0)
MC_NOISE_SEED_OFFSET = 1_000_000_001  # observation-noise RNG derived from trial_seed


@dataclass(frozen=True)
class MonteCarloTrialResult:
    """One full pipeline (comparison + selected run) outcome for MC aggregation."""

    success: bool
    miss_distance_m: float
    intercept_time_s: float
    switch_count: int


def sample_mc_target_initial_conditions(rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """Sample p0 in box and v_lin as random direction (downward z) times speed in [MC_IC_SPEED]."""
    p0 = np.array(
        [
            float(rng.uniform(MC_IC_POS_X[0], MC_IC_POS_X[1])),
            float(rng.uniform(MC_IC_POS_Y[0], MC_IC_POS_Y[1])),
            float(rng.uniform(MC_IC_POS_Z[0], MC_IC_POS_Z[1])),
        ],
        dtype=float,
    )
    speed = float(rng.uniform(MC_IC_SPEED[0], MC_IC_SPEED[1]))
    for _ in range(64):
        d = rng.standard_normal(3).astype(float)
        n = float(np.linalg.norm(d))
        if n < 1e-12:
            continue
        d = d / n
        if float(d[2]) < -0.08:
            return p0, d * speed
    d = np.array([0.75, 0.0, -0.66], dtype=float)
    d = d / float(np.linalg.norm(d))
    return p0, d * speed


def observe_target_position(
    p_true: np.ndarray,
    rng: np.random.Generator,
    std_m: float,
) -> np.ndarray:
    """p_obs = p_true + N(0, std^2 I); std <= 0 returns a copy of p_true (no noise)."""
    p = np.asarray(p_true, dtype=float)
    if float(std_m) <= 1e-15:
        return p.copy()
    return (p + rng.normal(0.0, float(std_m), size=3)).astype(float)


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
    rng: np.random.Generator,
    position_noise_std_m: float,
    replan_interval_s: float = 0.5,
    quiet: bool = False,
) -> tuple[np.ndarray, np.ndarray, float, bool, int]:
    """STEP 6–8: predictive + PN loop; replan checkpoints; optional single handoff by TTI margin (early window)."""
    s_i = float(best_spec.speed)
    p_true, v_t = target_position_velocity(p0, v_lin, 0.0, A_m, omega, m_axis)
    p_obs = observe_target_position(p_true, rng, position_noise_std_m)
    p_sel = np.asarray(best_spec.position0, dtype=float).copy()
    v_zero = np.zeros(3, dtype=float)
    u0 = desired_unit_direction(p_obs, v_t, p_sel, v_zero, s_i, dt, "predictive")
    v_sel = u0 * s_i

    tgt_hist: list[np.ndarray] = [p_true.copy()]
    sel_hist: list[np.ndarray] = [p_sel.copy()]

    committed_idx = int(committed_interceptor_idx)
    switch_count = 0
    has_switched = False
    t = 0.0
    hit = False
    next_replan_at = float(replan_interval_s)
    while t < max_time - 1e-12:
        t += dt
        p_true, v_t = target_position_velocity(p0, v_lin, t, A_m, omega, m_axis)
        p_obs = observe_target_position(p_true, rng, position_noise_std_m)
        _ = compute_predictive_direction(p_obs, v_t, p_sel, s_i)
        a_pn = compute_pn_acceleration(p_obs, v_t, p_sel, v_sel, PN_GAIN)
        v_sel = update_velocity_from_pn_lateral(v_sel, a_pn, s_i, max_turn_rate, dt)
        p_sel = p_sel + v_sel * dt
        tgt_hist.append(p_true.copy())
        sel_hist.append(p_sel.copy())
        while t + 1e-9 >= next_replan_at:
            t_log = next_replan_at
            best_now = _replanning_best_by_tti(p_obs, v_t, interceptor_candidates)
            if not quiet:
                if best_now is None:
                    print(f"[Replan t={t_log:.2f}s] Best: none (committed: I{committed_idx})")
                else:
                    bi, tti = best_now
                    print(f"[Replan t={t_log:.2f}s] Best: I{bi}, TTI={tti:.2f} (committed: I{committed_idx})")
            cur_spec = interceptor_candidates[committed_idx]
            cur_tti = time_to_intercept_cv(p_obs, v_t, cur_spec.position0, float(cur_spec.speed))
            if best_now is not None:
                bi, new_tti = best_now
                if bi != committed_idx:
                    cur_for_rule = float("inf") if cur_tti is None else float(cur_tti)
                    if (
                        (not has_switched)
                        and (t_log < float(SWITCH_ALLOW_UNTIL_S))
                        and (float(new_tti) + float(SWITCH_TTI_MARGIN_S) < cur_for_rule)
                    ):
                        if not quiet:
                            print(f"[SWITCH t={t_log:.2f}s] I{committed_idx} -> I{bi}")
                        committed_idx = bi
                        p_sel = np.asarray(interceptor_candidates[committed_idx].position0, dtype=float).copy()
                        u_sw = desired_unit_direction(p_obs, v_t, p_sel, v_zero, s_i, dt, "predictive")
                        v_sel = u_sw * s_i
                        has_switched = True
                        switch_count += 1
            next_replan_at += float(replan_interval_s)
        if float(np.linalg.norm(p_sel - p_true)) < float(range_threshold_m):
            hit = True
            break

    return np.stack(tgt_hist), np.stack(sel_hist), t, hit, switch_count


def run_sim(
    *,
    quiet: bool = False,
    noise_rng_seed: int | None = None,
    p0: np.ndarray | None = None,
    v_lin: np.ndarray | None = None,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    bool,
    float,
    float,
    tuple[int, InterceptorSpec, float] | None,
    list[InterceptorSpec],
    np.random.Generator,
]:
    dt = 0.05
    max_time = 20.0
    r_int = 1.0
    max_turn_rate = np.deg2rad(float(MAX_TURN_RATE_DEG_S))
    s_i = 18.0

    if p0 is None:
        p0 = np.array([0.0, 0.0, 20.0], dtype=float)
    else:
        p0 = np.asarray(p0, dtype=float).copy()
    if v_lin is None:
        v_lin = np.array([8.0, 0.0, -1.0], dtype=float)
    else:
        v_lin = np.asarray(v_lin, dtype=float).copy()
    A_m = TARGET_MANEUVER_A_M
    omega = TARGET_MANEUVER_OMEGA_RAD_S
    m_axis = TARGET_MANEUVER_AXIS

    seed = int(TARGET_POSITION_NOISE_RNG_SEED if noise_rng_seed is None else noise_rng_seed)
    rng = np.random.default_rng(seed)
    if not quiet:
        print("\n=== Noise Settings ===")
        print(f"Target position noise std: {TARGET_POSITION_NOISE_STD_M} m")
        print("(Velocity noise off; ground-truth target trajectory for history, dual-stop, and intercept metrics.)")

    p_true, v_t = target_position_velocity(p0, v_lin, 0.0, A_m, omega, m_axis)
    p_obs = observe_target_position(p_true, rng, TARGET_POSITION_NOISE_STD_M)
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
    if not quiet:
        print("\n=== Interceptor Feasibility (t = 0) ===\n")
        print(f"{'ID':<6}{'Feasible':<12}{'TTI (s)'}")
        print("--------------------------------")
    candidate_eval: list[tuple[int, InterceptorSpec, bool, float | None]] = []
    for idx, spec in enumerate(interceptor_candidates):
        feasible = feasibility_cv(p_obs, v_t, spec.position0, spec.speed)
        t_hit = time_to_intercept_cv(p_obs, v_t, spec.position0, spec.speed)
        candidate_eval.append((idx, spec, feasible, t_hit))
        if not quiet:
            tti_str = f"{t_hit:.2f}" if t_hit is not None else "None"
            print(f"I{idx}    {str(feasible):<12}{tti_str}")
    if not quiet:
        print()

    # STEP 5: select best by minimum TTI among feasible; PN excluded from pool (predictive/naive baseline only).
    eligible = [
        (idx, spec, t_hit)
        for idx, spec, feasible, t_hit in candidate_eval
        if feasible and t_hit is not None and spec.guidance != "pn"
    ]
    if not quiet:
        print("=== Best Interceptor Selection (t = 0) ===")
        print("Selection pool: Predictive / Naive only")
        print("Excluded from selection: PN")
        print()
    best_selection: tuple[int, InterceptorSpec, float] | None
    if not eligible:
        best_selection = None
        if not quiet:
            print("Selected ID: (none)")
            print("TTI: N/A")
            print("Position: N/A")
    else:
        best_idx, best_spec, best_t_hit = min(eligible, key=lambda r: r[2])
        best_selection = (best_idx, best_spec, best_t_hit)
        if not quiet:
            print(f"Selected ID: I{best_idx}")
            print(f"TTI: {best_t_hit:.2f}")
            print(f"Position: {best_spec.position0}")
    if not quiet:
        print()

        print("Interceptor positions:")
        for idx, spec in enumerate(interceptor_candidates):
            print(f"I{idx}: {spec.position0}")
        print("Target state (ground truth):")
        print("p_true:", p_true)
        print("p_obs (noisy, for CV / guidance):", p_obs)
        print("v_t:", v_t)
        for idx, spec in enumerate(interceptor_candidates):
            d = float(np.linalg.norm(p_true - spec.position0))
            print(f"I{idx} distance (to true target): {d}")

    v_zero = np.zeros(3, dtype=float)
    # Per-site initial velocity (speed s_i); each branch must use its own launch position, not p_n only.
    u_n = desired_unit_direction(p_obs, v_t, p_n, v_zero, s_i, dt, "naive")
    v_n = u_n * s_i

    u_p = desired_unit_direction(p_obs, v_t, p_p, v_zero, s_i, dt, "predictive")
    v_p = u_p * s_i

    # PN: same bootstrap contract as predictive — CV lead at p_k if available, else naive LOS (inside desired_unit_direction).
    u_k = desired_unit_direction(p_obs, v_t, p_k, v_zero, s_i, dt, "predictive")
    v_k = u_k * s_i

    tgt_hist: list[np.ndarray] = [p_true.copy()]
    naive_hist: list[np.ndarray] = [p_n.copy()]
    pred_hist: list[np.ndarray] = [p_p.copy()]
    pn_hist: list[np.ndarray] = [p_k.copy()]

    t = 0.0
    success = False

    while t < max_time - 1e-12:
        t += dt
        p_true, v_t = target_position_velocity(p0, v_lin, t, A_m, omega, m_axis)
        p_obs = observe_target_position(p_true, rng, TARGET_POSITION_NOISE_STD_M)

        u_naive = desired_unit_direction(p_obs, v_t, p_n, v_n, s_i, dt, "naive")
        v_n = update_velocity(v_n, u_naive, s_i, max_turn_rate, dt)
        p_n = p_n + v_n * dt

        u_pred = desired_unit_direction(p_obs, v_t, p_p, v_p, s_i, dt, "predictive")
        v_p = update_velocity(v_p, u_pred, s_i, max_turn_rate, dt)
        p_p = p_p + v_p * dt

        # PN: commanded accel from guidance.pn → lateral component in velocity frame → Euler step
        # → direction at constant |v| is implicit in update_velocity_from_pn_lateral → same turn-rate cap as others
        a_pn = compute_pn_acceleration(p_obs, v_t, p_k, v_k, PN_GAIN)
        v_k = update_velocity_from_pn_lateral(v_k, a_pn, s_i, max_turn_rate, dt)
        p_k = p_k + v_k * dt

        tgt_hist.append(p_true.copy())
        naive_hist.append(p_n.copy())
        pred_hist.append(p_p.copy())
        pn_hist.append(p_k.copy())

        d_n = float(np.linalg.norm(p_n - p_true))
        d_p = float(np.linalg.norm(p_p - p_true))
        if d_n < r_int and d_p < r_int:
            success = True
            break

    T = np.stack(tgt_hist)
    N = np.stack(naive_hist)
    P = np.stack(pred_hist)
    K = np.stack(pn_hist)

    return T, N, P, K, success, t, dt, best_selection, interceptor_candidates, rng


def run_single_pipeline_trial(*, trial_seed: int, quiet: bool = True) -> MonteCarloTrialResult:
    """One full ``run_sim`` + selected closed-loop; randomized ICs + derived observation-noise seed."""
    ic_rng = np.random.default_rng(int(trial_seed))
    p0_t, v_lin_t = sample_mc_target_initial_conditions(ic_rng)
    noise_seed = int(trial_seed) + int(MC_NOISE_SEED_OFFSET)
    T, N, P, K, _success, _t_elapsed, dt, best_selection, interceptor_candidates, rng = run_sim(
        quiet=quiet,
        noise_rng_seed=noise_seed,
        p0=p0_t,
        v_lin=v_lin_t,
    )
    dt_s = float(dt)
    th = INTERCEPT_EVAL_THRESHOLD_M
    if best_selection is None:
        return MonteCarloTrialResult(False, float("nan"), float("nan"), 0)
    sel_idx, sel_spec, _ = best_selection
    A_s = TARGET_MANEUVER_A_M
    om_s = TARGET_MANEUVER_OMEGA_RAD_S
    ax_s = TARGET_MANEUVER_AXIS
    max_t_s = 20.0
    max_turn_s = np.deg2rad(float(MAX_TURN_RATE_DEG_S))
    r_hit = 1.0
    T_sel, S_sel, _t_sel, _hit, switch_count = run_selected_interceptor_closed_loop(
        p0=p0_t,
        v_lin=v_lin_t,
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
        rng=rng,
        position_noise_std_m=TARGET_POSITION_NOISE_STD_M,
        replan_interval_s=0.5,
        quiet=quiet,
    )
    ev_sel = evaluate_intercept(T_sel, S_sel, dt_s, th)
    return MonteCarloTrialResult(
        success=bool(ev_sel[2]),
        miss_distance_m=float(ev_sel[0]),
        intercept_time_s=float(ev_sel[1]),
        switch_count=int(switch_count),
    )


def run_monte_carlo(num_trials: int = 50) -> None:
    """STEP A1–A2: repeat full offline pipeline; randomized target ICs + noise; aggregate selected-run metrics."""
    print("\n=== Scenario Settings ===")
    print(
        "Position range: "
        f"x in [{MC_IC_POS_X[0]}, {MC_IC_POS_X[1]}], "
        f"y in [{MC_IC_POS_Y[0]}, {MC_IC_POS_Y[1]}], "
        f"z in [{MC_IC_POS_Z[0]}, {MC_IC_POS_Z[1]}] m"
    )
    print(
        "Speed range: "
        f"[{MC_IC_SPEED[0]}, {MC_IC_SPEED[1]}] m/s "
        "(uniform magnitude; random 3D direction with vz < -0.08 after normalization)"
    )
    results = [
        run_single_pipeline_trial(trial_seed=MC_TRIAL_SEED_BASE + k, quiet=True)
        for k in range(int(num_trials))
    ]
    misses = np.array([r.miss_distance_m for r in results], dtype=float)
    times = np.array([r.intercept_time_s for r in results], dtype=float)
    switches = np.array([float(r.switch_count) for r in results], dtype=float)
    n_ok = int(sum(1 for r in results if r.success))
    rate_pct = 100.0 * float(n_ok) / float(num_trials) if num_trials > 0 else 0.0
    avg_miss = float(np.nanmean(misses))
    avg_time = float(np.nanmean(times))
    avg_sw = float(np.mean(switches))
    min_miss = float(np.nanmin(misses))
    max_miss = float(np.nanmax(misses))
    print("\n=== Monte Carlo Summary ===")
    print(f"Trials: {num_trials}")
    print(f"Success Rate: {rate_pct:.1f} %")
    print(f"Average Miss Distance: {avg_miss:.3f} m")
    print(f"Average Intercept Time: {avg_time:.3f} s")
    print(f"Average Switch Count: {avg_sw:.3f}")
    print(f"Min Miss Distance: {min_miss:.3f} m")
    print(f"Max Miss Distance: {max_miss:.3f} m")


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
    T, N, P, K, success, t_elapsed, dt, best_selection, interceptor_candidates, rng = run_sim(quiet=False)

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

    # STEP 6–8: closed-loop with replan + optional TTI-margin handoff between candidates.
    if best_selection is not None:
        sel_idx, sel_spec, _ = best_selection
        p0_s = np.array([0.0, 0.0, 20.0], dtype=float)
        v_lin_s = np.array([8.0, 0.0, -1.0], dtype=float)
        A_s = TARGET_MANEUVER_A_M
        om_s = TARGET_MANEUVER_OMEGA_RAD_S
        ax_s = TARGET_MANEUVER_AXIS
        dt_s = 0.05
        max_t_s = 20.0
        max_turn_s = np.deg2rad(float(MAX_TURN_RATE_DEG_S))
        r_hit = 1.0
        print("\n=== Selected Interceptor Run ===")
        print(f"Selected ID: I{sel_idx}")
        T_sel, S_sel, _t_sel, _, switch_count = run_selected_interceptor_closed_loop(
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
            rng=rng,
            position_noise_std_m=TARGET_POSITION_NOISE_STD_M,
            replan_interval_s=0.5,
            quiet=False,
        )
        ev_sel = evaluate_intercept(T_sel, S_sel, dt_s, th)
        print(f"Miss Distance: {ev_sel[0]:.3f} m")
        print(f"Intercept Time: {ev_sel[1]:.2f} s")
        print(f"Success: {ev_sel[2]}")
        print(f"Switch Count: {switch_count}")
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

    run_monte_carlo(50)

    plt.show()


if __name__ == "__main__":
    main()
