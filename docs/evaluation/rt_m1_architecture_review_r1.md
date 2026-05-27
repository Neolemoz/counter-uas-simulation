# RT-M1 — Architecture Review R1

**Phase:** PLAN-RT-M1 — multi-session architecture (docs only)  
**Prerequisite:** PLAT-RT-R3a session manager decomposition frozen

Plan: [rt_m1_multi_session_architecture_plan.md](../platform/rt_m1_multi_session_architecture_plan.md)  
Governance review: [rt_m1_governance_review_r1.md](rt_m1_governance_review_r1.md)

This review evaluates the proposed multi-session architecture against the current single-session codebase baseline.

---

## 1. Current baseline assessment

| Component | Current state | M1 proposal |
|-----------|---------------|-------------|
| `BridgeSessionManager._session` | Single optional slot | `SessionRegistry` dict (M2) |
| `start_session` guard | Rejects if prior non-terminal active | Capacity check `< 3` |
| `max_concurrent_sessions` | Constant=1, unwired | Constant=3, to be wired in M2 |
| `TelemetrySubscriptionStore` | Dict keyed by session_id | Already compatible |
| `ros_domain_id_for_session()` | Per-id derivation | Ready for isolation |
| `useRtSession.ts` | Single session hook | Multi-session workspace hook (M2) |
| `session_teardown.py` | Per-session composed paths | Reuse unchanged |

**Verdict:** Partial multi-session shapes exist; structural break is at manager slot + UI hook.

---

## 2. Session registry review

| Criterion | Result | Notes |
|-----------|--------|-------|
| Registry ownership clear | **Pass** | Bridge owns; UI owns selection |
| Capacity enforcement defined | **Pass** | `SESSION_CAPACITY_EXCEEDED` |
| Terminal eviction policy | **Pass** | Post-teardown removal |
| Per-session timeout independence | **Pass** | `tick_timeouts` per session |
| ROS domain isolation | **Pass** | Existing helper documented |
| Thread safety addressed | **Pass** | Registry mutex in M2 guidance |

---

## 3. Telemetry routing review

| Criterion | Result | Notes |
|-----------|--------|-------|
| Pull API unchanged | **Pass** | Session-scoped already |
| Active vs background rate tiers | **Pass** | 10 Hz / 1 Hz |
| Mirror isolation | **Pass** | Per SessionRecord |
| No cross-session channel merge | **Pass** | UI snapshot maps keyed by sessionId |
| Per-session rate limiter needed | **Pass** | M2 prerequisite documented |
| Global `_rate_ok` must split | **Pass** | Noted in telemetry contract §6 |

---

## 4. Editing ownership review

| Criterion | Result | Notes |
|-----------|--------|-------|
| Bridge-enforced editing lock | **Pass** | Recommended over UI-only |
| `EDITING_SESSION_MISMATCH` error | **Pass** | Defined in bridge contract §11 |
| Dual-surface (SVG/Cesium) binding | **Pass** | Same editing session |
| Local mirror per-session scoping | **Pass** | M2 requirement documented |
| Background read-only rule | **Pass** | UI + bridge gate |

---

## 5. Capture / handoff review

| Criterion | Result | Notes |
|-----------|--------|-------|
| Per-session capture from `stopped` | **Pass** | Unchanged lifecycle rule |
| No cross-session world read | **Pass** | Target SessionRecord only |
| Staging path isolation | **Pass** | Per capture_candidate_id |
| Global `max_staged_captures` shared | **Pass** | Documented |
| Handoff per capture_candidate_id | **Pass** | SA1 bridge unchanged |

---

## 6. Failure isolation review

| Scenario | Expected behavior | Documented |
|----------|-------------------|------------|
| Session A adapter crash | A → failed; B,C continue | **Pass** |
| Session A timeout | A auto-cleanup only | **Pass** |
| Session A at entity cap | A rejects; B unaffected | **Pass** |
| Registry at capacity | New start rejected | **Pass** |
| Editing session evicted | Lock reassigned | **Pass** |

---

## 7. Risk mitigations

| Risk ID | Risk | Mitigation | Result |
|---------|------|------------|--------|
| R7 (S1) | Scope creep to distributed multi-session | Single-bridge cap=3 explicit | **Pass** |
| M1-R1 | Cross-session entity ID confusion | Session prefix in diagnostics | **Pass** |
| M1-R2 | Resource exhaustion (3× Gazebo) | Mock-default; live opt-in guidance | **Pass** |
| M1-R3 | Editing wrong session | Bridge lock + tab indicator | **Pass** |
| M1-R4 | Capture cross-contamination | Target session world only | **Pass** |
| M1-R5 | Concurrent HTTP race on registry | Registry mutex (M2) | **Pass** |

---

## 8. PLAT-RT-M2 prerequisites summary

### Bridge

1. `SessionRegistry` replacing `_session`
2. Wire `max_concurrent_sessions=3`
3. Per-session `RateLimiter` and `tick_timeouts`
4. `list_sessions`, `set_editing_session` commands
5. Thread-safe registry access
6. Multiple `RuntimeHandle` instances
7. Integration tests: capacity, editing lock, capture isolation, 3 concurrent

### UI

1. Multi-session workspace hook (refactor `useRtSession.ts`)
2. Session tab component
3. Per-session snapshot/local mirror maps
4. Background diagnostics accordion

### Contracts/tests

1. Bridge contract §11 implemented
2. `test_rt_sandbox_bridge.py` multi-session scenarios

---

## 9. Verdict

**Pass — architecture suitable for PLAN-RT-M1 docs freeze. PLAT-RT-M2 may proceed after M1 freeze audit.**

---

## Related

- [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md)
- [rt_roadmap_m1_m2_v1.md](rt_roadmap_m1_m2_v1.md)
