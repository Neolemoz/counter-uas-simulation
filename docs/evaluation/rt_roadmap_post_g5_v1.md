# RT Post-G5 Consolidation Roadmap (`rt_roadmap_post_g5_v1`)

**Phase:** PLAN-RT-R1 — consolidation roadmap (docs only; **not authorized** as implementation)  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](rt_r1_architecture_stabilization_review_r1.md)  
**Prerequisite:** PLAT-RT-G5 frozen; PLAN-RT-R1 frozen

This document tiers follow-on work after the G5 stop line. **No wave listed here may start without** its own plan, governance review, and freeze audit.

---

## Forbidden until explicit new wave audit

| Theme | Rationale |
|-------|-----------|
| Automatic SA replay ingestion | G5 + R1 stop line |
| Federation / corpus writes from RT | `rt_sa_export_boundary_v1` |
| SA viewer live session / runtime hooks | PLAN-RT-S1 boundary |
| Distributed multi-bridge | Conflicts with local prototype — **local multi-session (≤3) authorized by PLAN-RT-M1** on single bridge only |
| Autonomous runtime behavior | Not in RT frontier |
| Production security / cloud infra | Prototype local-only |
| Full telemetry UI / Cesium runtime viz | **Closed by PLAT-RT-T1** (telemetry) and **PLAT-RT-T3** (Cesium) |

---

## P0 — Blocker before any RT expansion

**Status:** Closed by **PLAT-RT-R1a** (frozen). See [rt_r1a_freeze_audit.md](rt_r1a_freeze_audit.md).

| Wave | Finding IDs | Objective | Status |
|------|-------------|-----------|--------|
| **RT-R1a** (combined) | R1-SYNC-01, R1-AUTH-02, R1-GOV-02, R1-AUDIT-02, R1-SA-03 | Revision glossary, authority labels, audit `event_kind` | **Closed** |

---

## P1 — Recommended before UI / Cesium / SA bridge

**Next action after PLAN-RT-R2f freeze:** Expansion only with explicit new wave audit. **P2 maintenance complete** (PLAT-RT-R3d).

| Wave (provisional) | Finding IDs | Objective | Status |
|--------------------|-------------|-----------|--------|
| **RT-R2a Adapter poll unification** | R1-SYNC-03 | Single post-mutation adapter tick (feedback + telemetry) after entity/template ops | **Closed by PLAT-RT-R1b** |
| **RT-R2b Stale utility consolidation** | R1-SYNC-02, R1-DEBT-03 | Shared UTC age helper for G3/G4 mirrors | **Closed by PLAT-RT-R1b** |
| **RT-R2c Telemetry path decision** | R1-DEBT-02 | Deprecate or wire `TelemetryBuffer` heartbeats into subscription model; one maintainer doc | **Partial — closed by PLAT-RT-R1b** (docs boundary; buffer retained) |
| **RT-R2d Template adapter resync** | R1-AUTH-04 | Explicit resync policy after `apply_runtime_template` | **Closed by PLAT-RT-R2d** |
| **RT-R2e Capture pose cognition** | R1-CAP-02 | Reviewer-facing doc for tri-source pose history interpretation | **Closed by PLAT-RT-R2e** |
| **RT-R2f SA bridge planning** | R1-SA-05 | Scoped plan for manual SA import only (if ever authorized) | **Closed by PLAN-RT-R2f** |

---

## P2 — Maintenance / entropy reduction

| Wave (provisional) | Finding IDs | Objective |
|--------------------|-------------|-----------|
| **RT-R3a Session manager decomposition** | R1-DEBT-01 | Extract capture, adapter, telemetry modules without behavior change | **Closed by PLAT-RT-R3a** |
| **RT-R3b Lifecycle doc hardening** | R1-LIFE-02 | Expand `bridge_disconnected` transition tests and doc | **Closed by PLAT-RT-R3b** |
| **RT-R3c Subcommand governance lint** | R1-GOV-04 | CI check that `RUNTIME_SUBCOMMANDS` ⊆ implemented handlers | **Closed by PLAT-RT-R3c** |
| **RT-R3d World revision hint policy** | R1-SYNC-05 | Document when hint may diverge from `world.revision` | **Closed by PLAT-RT-R3d** |

---

## Expansion waves (explicitly out of R1 scope)

These require **separate** plans after P0 closure:

| Theme | Notes |
|-------|-------|
| RT telemetry UI | Loopback pull consumer; mirror banners mandatory — **Closed by PLAT-RT-T1** |
| RT world editing UI (RT-T2) | Drag/drop entity editing in `platform/rt-sandbox-ui/` — **Closed by PLAT-RT-T2** |
| Cesium runtime visualization | Must not treat mirrors as replay truth — **Closed by PLAT-RT-T3** |
| SA replay bridge | Manual approval path only — **Closed by PLAT-RT-SA1** |
| Multi-runtime orchestration | Conflicts with single-session prototype — **superseded by PLAN-RT-M1** for local ≤3 sessions |
| Live Gazebo default-on | Governance constant change + new freeze audit |

---

## Finding index (quick reference)

| ID | Tier |
|----|------|
| R1-SYNC-01 | P0 |
| R1-AUTH-02, R1-GOV-02, R1-AUDIT-02, R1-SA-03 | P0 |
| R1-SYNC-02, R1-SYNC-03, R1-CAP-02, R1-AUTH-04, R1-DEBT-02, R1-DEBT-03, R1-SA-05 | P1 |
| R1-LIFE-02, R1-DEBT-01, R1-SYNC-05, R1-GOV-04 | P2 |

---

## Stop line

P2 maintenance complete (PLAT-RT-R3d). **PLAT-RT-T1**–**T5** and **PLAT-RT-SA1** (manual RT→SA import) frozen. **PLAN-RT-M1** (local multi-session architecture, docs only) authorized — see [rt_roadmap_m1_m2_v1.md](rt_roadmap_m1_m2_v1.md). PLAT-RT-M2 implementation requires M1 freeze audit. Deeper SA workflow integration and bridge expansion still require explicit new wave audit.
