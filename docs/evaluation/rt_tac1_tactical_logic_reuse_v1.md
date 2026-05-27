# RT Tactical Logic Reuse (`rt_tac1_tactical_logic_reuse_v1`)

**Phase:** PLAN-RT-TAC1 — tactical controller architecture (docs only)  
**Authority:** [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md); [rt_tac1_tactical_controller_architecture_plan.md](../platform/rt_tac1_tactical_controller_architecture_plan.md)

Defines how future RT sandbox code may **reuse** existing counter-UAS tactical algorithms safely. TAC1 does **not** import or call engine code from the RT bridge.

---

## 1. Reference implementation map

| Concern | Primary file | Key symbols |
|---------|--------------|-------------|
| CV intercept solver | `src/gazebo_target_sim/gazebo_target_sim/guidance_lib.py` | `solve_intercept_time`, `compute_intercept`, `compensate_target_for_delay` |
| Selection + assignment | `src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py` | `_update_committed_selection`, `_on_control_multi`, `_pick_best_feasible_interceptor_single` |
| TTI at speed cap | Same | `_intercept_feasibility_triple`, `_feasibility_tti_for_assignment` |
| Switch hysteresis | Same | `selection_margin_s`, `switch_window_s`, `_stabilize_multi_assignment` |
| Assignment lock | Same | `_assigned_interceptor_id`, `_assignment_lock_duration` (1.5 s) |
| Feasibility | Same | `FeasibilityDecision`, `_tti_feasible`, `is_intercept_feasible` |
| Observability | Same | `_obs_line` → `[TACTICAL_*]` tags |
| Offline mirror | `simulation/defense_types.py`, `simulation/core/intercept.py` | `time_to_intercept_cv`, `feasibility_cv` |
| Golden tests | `src/counter_uas/test/test_intercept_solver_golden.py`, `test_guidance_lib.py` | Regression for solver |

**Review doc:** [gnc_intercept_review.md](../gnc_intercept_review.md) — solver vs plant dynamics gap (informational for RT sandbox expectations).

---

## 2. Intercept solver reuse

### 2.1 Contract

`solve_intercept_time(r0, v_target, interceptor_speed) -> float | None` — smallest \(t > 0\) with closing geometry satisfied within tolerance.

`compute_intercept(...) -> (t, p_hit, u_hat) | None` — hit point and unit direction.

### 2.2 RT consumption rules (PLAT-RT-TAC2+)

| Rule | Rationale |
|------|-----------|
| Prefer **vendored copy** or explicit submodule import of `guidance_lib` only | Avoid ROS node coupling |
| No new parser-visible fields | Parser safety |
| Unit-test against golden vectors from `test_intercept_solver_golden.py` | Regression parity |
| Document speed cap used for ranking | Match engine `t_intercept_at_cap` semantics |

---

## 3. TTI ranking reuse

### 3.1 Engine behavior (reference)

- **Single-target:** Among feasible interceptors, pick **minimum TTI** (tie-break by stable id index) unless `dome_sel_mode == 'nearest'`.
- **Multi-target:** Greedy assign by threat score; per-row minimum TTI; stabilize with `assignment_switch_tti_margin_s`.
- **Ranking metric:** Prefer **TTI at `interceptor_max_speed`** (`t_intercept_at_cap`) for assignment cost — not minimum closing speed from bisection alone.

### 3.2 RT consumption rules

| Rule | Rationale |
|------|-----------|
| RT sandbox ranking uses **same cap-speed TTI** definition | Comparable cognition UI |
| Switch requires `candidate_tti + margin < current_tti` **and** dwell (`switch_window_s`) | Avoid flicker in Assisted/Autonomous |
| `nearest` mode is **opt-in** sandbox config — default `tti` | Document in session template metadata (future) |
| Rankings exposed as **explanatory** until mode commits assignment | Authority model |

---

## 4. Assignment lock reuse

### 4.1 Engine behavior (reference)

After commit, **1.5 s** lock holds assignment; `[TACTICAL_SWITCH] decision=hold reason=assignment_lock_active`; `[TACTICAL_ASSIGNMENT_LOCK]` on set.

### 4.2 RT consumption rules

| Mode | Lock behavior |
|------|---------------|
| Manual | Optional user-visible lock after assign — user may override |
| Assisted | Lock **must not** bypass pending approval — lock applies only after `user_approval_authoritative` commit |
| Autonomous | Lock may mirror engine policy for stability |

RT lock state is `assignment_lock_active` telemetry — `explanatory_telemetry` unless Autonomous commit in flight.

---

## 5. Feasibility reuse

### 5.1 Engine reject reasons (reference)

`no_intercept_solution_in_window`, `required_min_speed_above_cap`, `cap_solution_outside_time_window`, `feasible`.

### 5.2 RT consumption rules

| Rule | Rationale |
|------|-----------|
| Surface feasibility in UI as **review hints** only | Not engage authority |
| Map to `tactical_health` summary — not weapon readiness | Lexicon |
| Do not gate bridge entity commands on feasibility in Manual | User drives sandbox |
| Assisted may **recommend** against infeasible candidates — still requires approval | Mode contract |

---

## 6. Observability: `[TACTICAL_*]` vs RT mirrors

Per [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md) and `scripts/evaluation/README.md`:

| Layer | Role |
|-------|------|
| Engine topics `/interceptor/selected_id`, `assigned_target` | Authoritative for **engine/replay tooling** when node runs |
| `[TACTICAL_COMMIT]`, `[TACTICAL_SWITCH]`, `[TACTICAL_FEASIBILITY]`, etc. | **Explanatory evidence** — parsers must not depend without governance review |
| `=== Interceptor Selection ===` blocks | Parser-visible selection audit (evaluation oracle) |
| RT tactical mirrors (`selected_candidate_id`, etc.) | RT-defined; authority per [rt_tac1_tactical_modes_v1.md](rt_tac1_tactical_modes_v1.md) |

### 6.1 Tag reference (engine — do not emit from RT bridge in TAC1)

| Tag | Meaning (explanatory) |
|-----|----------------------|
| `[TACTICAL_COMMIT]` | Committed interceptor change |
| `[TACTICAL_SELECTED_ID]` | Topic publish transition |
| `[TACTICAL_SWITCH]` | Hold/switch evaluation |
| `[TACTICAL_FEASIBILITY]` | Periodic feasible snapshot |
| `[TACTICAL_FEASIBILITY_REJECT]` | Per-interceptor reject |
| `[TACTICAL_ASSIGNMENT_LOCK]` | Lock window started |
| `[TACTICAL_MULTI_ASSIGN]` | Multi-target assignment decision |
| `[TACTICAL_ASSIGNED_TARGET]` | Per-interceptor assigned target topic |

RT PLAT waves may emit **parallel** RT audit `event_kind: tactical` — not raw log tag passthrough.

---

## 7. Safe consumption checklist (PLAT implementers)

1. No import of `rclpy` or `interception_logic_node` in bridge hot path.
2. Pure functions + numpy only in tactical controller package.
3. No subscription to `/tracks/state` or weapon topics from RT ([rt_bridge_contract_v1.md](rt_bridge_contract_v1.md)).
4. Engine logs may be **ingested optionally** as read-only diagnostic input — never authority.
5. SA export includes RT tactical timeline only after TAC5 normalization — `replay_boundary_scoped`.

---

## 8. Out of scope

- Changing engine node parameters or topics
- Parser contract changes
- Automatic port of `guidance_kernel` PN execution to RT

---

## Related

- [rt_tac1_tactical_telemetry_v1.md](rt_tac1_tactical_telemetry_v1.md)
- [rt_tac1_tactical_capture_continuity_v1.md](rt_tac1_tactical_capture_continuity_v1.md)
- [gnc_intercept_review.md](../gnc_intercept_review.md)
