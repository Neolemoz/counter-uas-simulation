# RT — Next Frontiers Roadmap v3

**Phase:** Advisory roadmap after PLAN-RT-F7 freeze  
**Prerequisite:** PLAN-RT-F7 frozen; PLAT-RT-M3 complete; PLAT-RT-F6 P0–P2 frozen  
**Supersedes (advisory ranking only):** [rt_roadmap_next_frontiers_v2.md](rt_roadmap_next_frontiers_v2.md) — v2 history preserved  
**Master plan:** [rt_f7_post_f6_advisory_expansion_plan.md](../platform/rt_f7_post_f6_advisory_expansion_plan.md)

This document ranks **possible** next steps after F7 planning freeze. **None are authorized** until scoped PLAT plan + governance + contamination review + freeze audit per phase.

---

## 1. Vocabulary (unchanged from v2)

| Label | Meaning |
|-------|---------|
| **PLAN-RT-F7** | Post-F6 advisory expansion — **docs frozen** |
| **PLAT-RT-F7** | Implementation backlog — P0–P2 in [rt_roadmap_plat_rt_f7_v1.md](rt_roadmap_plat_rt_f7_v1.md) |
| **Distributed multi-bridge** | **Forbidden** — not a numbered frontier |

---

## 2. Completed since v2

| ID | Summary | Status |
|----|---------|--------|
| PLAN-RT-M3 | Local multi-session polish plan | **docs frozen** |
| PLAT-RT-M3 P0–P2 | Session inspect, poll UX, tab reorder | **frozen — M3 complete** |
| PLAN-RT-F7 | Post-F6 advisory expansion | **docs frozen** (this wave) |

---

## 3. Recommended next major step

**Recommendation (advisory):** **Platform checkpoint review** — PLAT-RT-F7 complete (P0–P2 frozen)

**Rationale:**

1. **PLAT-RT-F7** trilogy delivered — queue/aggregation, triage UI, batch export v2, dry-run hardening.  
2. No P3 under F7 without new PLAN wave.  
3. Consolidation review validates advisory frontier closure before new expansion.

**Alternate:** Explicit **PLAN** wave for a future advisory frontier (not authorized by F7 freeze).

**Not recommended:** Distributed multi-bridge under any numeric frontier ID.

---

## 4. PLAT-RT-F7 phase summary

| Phase | Focus | Contamination |
|-------|--------|---------------|
| P0 | Schema + CLI + derive extensions | Fresh review before P0 freeze |
| P1 | Read-only triage UI | `tier0-rt-ui`; no action buttons |
| P2 | Bulk hardening + stand-up export | P2 re-audit |

See [rt_roadmap_plat_rt_f7_v1.md](rt_roadmap_plat_rt_f7_v1.md).

---

## 5. Explicit non-frontiers

Unchanged from v2 — parser/topic changes, operational HITL/C2, browser capture, SA viewer live RT hooks, federation authority from RT, PX4/MAVLink/hardware, distributed multi-bridge.

---

## 6. Wave authorization checklist

Before PLAT-RT-F7 implementation:

1. Scoped implementation plan in `docs/platform/` (`rt_plat_f7_p0_*`, etc.)
2. Contract compliance with F7 v1 docs
3. Governance review + contamination review per phase
4. Freeze audit + freeze registry row per PLAT phase
5. Regression: `lint_rt_runtime_subcommands`, batch advisory pytest, `tier0-rt-ui`

---

## 7. Stop line

**PLAN-RT-F7** freeze means: stop before PLAT implementation not meeting §6 checklist.
