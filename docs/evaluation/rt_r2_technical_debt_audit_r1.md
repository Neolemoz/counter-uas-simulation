# RT-R2 — Technical Debt Audit R1

**Phase:** PLAN-RT-R2 — platform maturity review  
**Plan:** [rt_r2_runtime_platform_maturity_plan.md](../platform/rt_r2_runtime_platform_maturity_plan.md)  
**Master review:** [rt_r2_platform_maturity_review_r1.md](rt_r2_platform_maturity_review_r1.md)

No runtime code was modified for this audit.

---

## 1. R1 finding closure matrix

| R1 ID | Original tier | Status after RT roadmap | Closed by |
|-------|---------------|-------------------------|-----------|
| R1-SYNC-01 | P0 | **Closed** | PLAT-RT-R1a — [rt_revision_vocabulary_v1.md](rt_revision_vocabulary_v1.md) |
| R1-AUTH-02 | P0 | **Closed** | PLAT-RT-R1a — [rt_authority_model_v1.md](rt_authority_model_v1.md), `authority_labels.py` |
| R1-GOV-02 | P0 | **Closed** | PLAT-RT-R1a + T1 banners; mirror `source` discipline in contracts |
| R1-AUDIT-02 | P0 | **Closed** | PLAT-RT-R1a — [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md) `event_kind` |
| R1-SA-03 | P0 | **Closed** | PLAT-RT-T1–T5, V1–V2, X1 governance chrome |
| R1-SYNC-02 | P1 | **Closed** | PLAT-RT-R1b — `time_utils.py`, shared stale helpers |
| R1-SYNC-03 | P1 | **Closed** | PLAT-RT-R1b — [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md), `adapter_poll.py` |
| R1-DEBT-03 | P1 | **Closed** | PLAT-RT-R1b |
| R1-CAP-02 | P1 | **Closed** | PLAT-RT-R2e — [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md) |
| R1-AUTH-04 | P1 | **Closed** | PLAT-RT-R2d — [rt_template_resync_policy_v1.md](rt_template_resync_policy_v1.md) |
| R1-DEBT-02 | P1 | **Partial** | PLAT-RT-R1b — subscription path primary; `TelemetryBuffer` retained for heartbeat boundary |
| R1-SA-05 | P1 | **Closed** | PLAN-RT-R2f, PLAT-RT-SA1–SA3 |
| R1-DEBT-01 | P2 | **Closed** | PLAT-RT-R3a — facade ~580 LOC + `session_*` handlers |
| R1-LIFE-02 | P2 | **Closed** | PLAT-RT-R3b — [rt_lifecycle_transitions_v1.md](rt_lifecycle_transitions_v1.md) |
| R1-SYNC-05 | P2 | **Closed** | PLAT-RT-R3d — [rt_world_revision_hint_policy_v1.md](rt_world_revision_hint_policy_v1.md) |
| R1-GOV-04 | P2 | **Closed** | PLAT-RT-R3c — subcommand lint + registry |
| R1-LIFE-04 | P1 | **Residual** | Template apply does not auto-reset workflow — documented, acceptable |
| R1-DEBT-04 | P2 | **Mitigated** | Roadmap delivered under wave discipline; entropy risk shifts to UI surface area |

---

## 2. Hotspots (current)

| Hotspot | Scale | Risk | Notes |
|---------|-------|------|-------|
| `App.tsx` | ~628 LOC | **Medium** | Orchestrates sessions, Cesium, tactical, experiment, handoff — primary UI integration point |
| `session_manager.py` facade | ~580 LOC | **Low–Med** | Post-R3a router; domain logic delegated to handlers |
| `tactical_controller.py` + handlers | Multi-module | **Low** | Isolated per-session; TAC2–5 frozen |
| Cesium bundle | Heavy dep | **Med** | Build ~316 KB JS (gzip ~92 KB); contributor onboarding cost |
| Experiment stack | `src/experiment/` + batch CLI | **Low** | Isolated; browser capture forbidden by isolation tests |

---

## 3. Ownership drift

Cross-walk of [rt_session_manager_ownership_v1.md](rt_session_manager_ownership_v1.md) vs implementation (May 2026):

| Concern | Documented owner | Drift |
|---------|------------------|-------|
| Lifecycle | `session_lifecycle_handlers.py` | **None** |
| Multi-session registry | `session_registry.py`, `session_registry_handlers.py` | **None** — cap=3 in `governance.py` |
| Tactical | `session_tactical_handlers.py`, `tactical_controller.py` | **None** |
| Capture | `session_capture_handler.py` | **None** |
| Telemetry pull | `session_telemetry_coordinator.py` | **None** |
| UI session workspace | `useRtSessionWorkspace.ts` | **None** — editing lock in UI + bridge |

**Verdict:** Ownership map matches post-R3a decomposition. Residual coupling is intentional facade routing, not undocumented logic migration.

---

## 4. Duplication inventory

| Area | Duplication | Severity | Recommendation (future wave) |
|------|-------------|----------|------------------------------|
| Telemetry cognition | UI `cognition.ts` helpers vs bridge audit event names | Low | Doc cross-ref only; no merge required |
| Pose truth | Registry + feedback mirror + telemetry mirror at capture | Low (intentional) | Keep tri-source; R2e docs sufficient |
| Experiment snapshot vs capture manifest | Overlapping `tactical_state` / `world_summary` fields | Low | Optional schema note in X1 contract |
| Revision counters | Multiple counters (documented in R1a) | Low | No new counter without contract wave |
| Stale helpers | Shared via R1b `time_utils` | **Closed** | — |

---

## 5. Test coverage map

### Bridge (pytest)

| Module | Primary test file | Coverage quality |
|--------|-------------------|------------------|
| Session / lifecycle | `test_rt_sandbox_bridge.py` | **Strong** — commands, timeouts, multi-session |
| Adapter / sync / telemetry | same | **Strong** — G2–G6 paths, mock default |
| Capture / normalization | same | **Strong** — G5, R2e, approval |
| Tactical TAC2–5 | same | **Strong** — modes, annex |
| Experiment batch | `test_rt_experiment_batch.py` | **Moderate** — dry-run + mocked manifest write |
| Subcommand lint | `lint_rt_runtime_subcommands.py` | **Gate** — tier0 |

Approximate RT pytest: **147+** tests in bridge + batch modules (May 2026).

### UI (Vitest)

| Area | Test files | Coverage quality |
|------|------------|------------------|
| Governance / isolation | `governance.test.ts`, `isolation.test.ts` | **Strong** — lexicon, SA import forbid |
| Experiment X1 | `experimentSchema`, `experimentCompare`, `TacticalAbCompareTable` | **Moderate** |
| Cesium / terrain V2 | `rtFictionalTerrain`, `terrainCognition`, layers | **Moderate** |
| Tactical panels | `TacticalManual/Assisted/Autonomous` | **Moderate** — render + lexicon |
| Multi-session | `useRtSessionWorkspace`, `SessionTabBar` | **Moderate** |
| Workstation / handoff | `captureHandoffCognition`, workflow | **Moderate** |

Approximate Vitest: **104** tests, **31** files (May 2026).

### Gaps (not blocking maturity plateau)

| Gap | Risk | Notes |
|-----|------|-------|
| Live Gazebo stack in CI | Med | Mock/stub default; G6 validated locally/maintainer |
| Browser E2E multi-session | Med | Unit/isolation tests; no Playwright harness |
| Full experiment batch integration | Low | Mock HTTP in pytest; maintainer runs live |
| SA3 viewer panel | Low | SA viewer tests separate; RT bridge supplies bundle field |
| Distributed multi-bridge | N/A | Forbidden until M3 audit |

---

## 6. Maintainability risks

| Risk | Tier | Mitigation today |
|------|------|------------------|
| Doc scatter (~40+ RT freeze/review docs) | P2 | [freeze_registry_r1.md](freeze_registry_r1.md), AGENTS.md index |
| `App.tsx` growth without decomposition | P1 | Stop line after R2; future wave may extract hooks/layout only |
| Zod experiment schema vs bridge JSON schemas | P2 | Separate concerns (local manifest vs bridge contract) |
| Cesium contributor ramp-up | P2 | T3/T5/V1/V2 contracts + fictional georef disclaimers |
| Misread of mirrors as operational truth | P1 | Frozen banners + cognition hub + R2 governance re-pass |

---

## 7. Debt verdict

| Category | Verdict |
|----------|---------|
| R1 P0/P1 closure | **Pass** — all addressed or explicitly residual (LIFE-04, partial DEBT-02) |
| Hotspot concentration | **Pass-with-conditions** — `App.tsx` is next maintenance candidate |
| Ownership | **Pass** |
| Test adequacy for plateau | **Pass** — gaps documented; no blocker for freeze |
| Expansion readiness | **Conditional** — see [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md) |

**Recommendation:** Accept technical debt posture at **maturity plateau**. Defer structural UI refactor unless a scoped maintenance wave is authorized post-R2.
