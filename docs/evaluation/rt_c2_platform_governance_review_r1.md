# RT-C2 — Platform Governance Review R1

**Phase:** PLAN-RT-C2 — runtime platform consolidation review  
**Plan:** [rt_c2_runtime_platform_consolidation_plan.md](../platform/rt_c2_runtime_platform_consolidation_plan.md)  
**Master review:** [rt_c2_platform_consolidation_review_r1.md](rt_c2_platform_consolidation_review_r1.md)  
**Freeze audit:** [rt_c2_platform_consolidation_freeze_audit.md](rt_c2_platform_consolidation_freeze_audit.md)  
**Baseline:** [rt_c1_platform_governance_review_r1.md](rt_c1_platform_governance_review_r1.md)

## Scope verdict

| Question | Answer |
|----------|--------|
| Docs-only wave? | Yes |
| SA viewer live hooks introduced? | No — F7 freeze: no `platform/sa-r0-viewer/` changes |
| Parser/topic changes? | No |
| New bridge commands from C2? | No |
| F7 changed default behavior? | No new defaults at consolidation freeze |
| F7 auto-import risk? | Re-checked — see §6 |
| Federation writes from RT? | No |

**Recommendation:** Freeze **PLAN-RT-C2** (docs frozen).

---

## 1. Authority boundaries

Re-audit against [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) and [rt_authority_model_v1.md](rt_authority_model_v1.md).

| Surface | Authoritative for sandbox? | Authoritative for SA replay? |
|---------|---------------------------|------------------------------|
| Entity registry / bridge commands | **Yes** (command truth) | No |
| Pose sync mirror | No — explanatory | No |
| Telemetry mirror / pull buffers | No — explanatory | No |
| Fidelity truth mirror (F5b) | No — explanatory | No |
| Advisory readiness / queue / triage (F6/F7) | No — explanatory | No |
| Batch review v2 export | No — maintainer artifact | No |
| Staging capture | No — pending approval | No |
| SA replay bundle | No | **Yes** (static bundle only) |

| Check | Result |
|-------|--------|
| Mirrors ≠ authority in UI | **Pass** |
| F7 triage panel has no commit/import actions | **Pass** |
| Fidelity ≠ operational sensor truth | **Pass** |
| Advisory ≠ import authority | **Pass** |
| Experiment metrics ≠ readiness scoring | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| C2-GOV-AUTH-01 | Pass — command registry remains sole mutation authority |
| C2-GOV-AUTH-02 | Pass — F7 queue/cohort labels are advisory cohorts only |

---

## 2. RT↔SA separation

| Check | Result |
|-------|--------|
| `platform/sa-r0-viewer/` not used for live RT | **Pass** |
| `reject_auto_sa_import` unchanged | **Pass** |
| Manual import only (SA1) | **Pass** |
| Handoff mirror read-only (SA2) | **Pass** |
| F7 does not invoke `rt_sa_import commit` from browser | **Pass** |
| `rt_sa_import_dry_run.py` preview-only (P2 hardened) | **Pass** |
| `rt_handoff_batch_advisory.py` without `--commit-all` | **Pass** |
| F7 v2 export `dry_run` always true | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| C2-GOV-SA-01 | Pass — RT writes `runs/rt_sandbox/` only |
| C2-GOV-SA-02 | Pass — capture ≠ import after F7 |
| C2-GOV-SA-03 | Pass — F7 extends maintainer cognition without shortening SA1 gates |

---

## 3. Replay boundaries

| Boundary | Enforcement |
|----------|-------------|
| RT live state ≠ replay authority | Banners, NOT SA REPLAY AUTHORITY |
| F3 annex UI ≠ SA scrubber | RT-local panels only |
| F7 triage ≠ SA corpus browser | RT-local queue only |
| SA3 tactical continuity | Read-only counts/timeline in bundle |

| Check | Result |
|-------|--------|
| F7/M3 do not load SA bundles in RT UI | **Pass** |
| Replay annotations explanatory only | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| C2-GOV-REP-01 | Pass |

---

## 4. Deny-by-default rules

| Gate | Implementation |
|------|----------------|
| Command allow-list | `ALLOWED_COMMANDS` |
| Browser forbidden calls | `isolation.test.ts` |
| Tactical forbidden lexicon | panel + governance tests |
| F2 experiment import guards | `experimentImportGuards.ts` |
| ROS topic allow-list | `ros_allowlist.py` |
| Runtime subcommands | R3c lint gate |
| F7 dry-run-review | CLI preview-only |

| Check | Result |
|-------|--------|
| No browser→ROS | **Pass** |
| No rosbridge / legacy web RT | **Pass** |
| No federation_register from RT UI | **Pass** |
| No hidden `--commit-all` on F7 paths | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| C2-GOV-DENY-01 | Pass |

---

## 5. Banners and lexicon

| Banner / label | Wave | Purpose |
|----------------|------|---------|
| `BANNER_EXPERIMENT` | X1/F1 | Compare explanatory |
| `BANNER_FIDELITY_TRUTH` | F5b | Sim-scoped truth, not ops |
| `BANNER_SA_WORKFLOW_ADVISORY` | F6/F7 | Advisory ≠ authority |
| T1 governance chrome | T1 | Mirror non-authority |
| Readiness grouping chips | F7 | Cohort labels — not ops readiness |

| Check | Result |
|-------|--------|
| No unqualified “ready for operations” copy | **Pass** |
| `import_ready` ≠ committed | **Pass** |
| Triage queue copy does not imply auto-import | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| C2-GOV-LEX-01 | Pass |

---

## 6. F7 P2 contamination re-check

Re-validated against [rt_plat_f7_p0_handoff_contamination_review_r1.md](rt_plat_f7_p0_handoff_contamination_review_r1.md), [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md), and [rt_plat_f7_p2_freeze_audit.md](rt_plat_f7_p2_freeze_audit.md).

| Landmine | Post-F7 P2 status |
|----------|-------------------|
| Browser corpus commit | **Not present** |
| `--commit-all` on batch paths | **Not present** |
| F7 v2 export writes without opt-in | **Mitigated** — preview file write opt-in only |
| `dry_run` false on v2 schema | **Not present** — always true |
| SA viewer live hooks | **Not present** — no sa-r0-viewer edits in F7 |
| Triage UI action buttons (import/commit) | **Not present** — read-only P1 |

| Finding ID | Verdict |
|------------|---------|
| C2-GOV-F7-01 | Pass — F7 shipped with contamination reviews on record |
| C2-GOV-F7-02 | Pass-with-conditions — any **PLAN-RT-F8** must repeat full contamination audit |

---

## 7. Maintainer-only surfaces

| Surface | Browser | Maintainer CLI |
|---------|---------|----------------|
| `capture_session` | Forbidden | bridge tests / batch CLIs |
| `rt_advisory_batch_review_v2` export | Copy JSON preview only | `standup-export`, `grouped-export`, `dry-run-review` |
| `rt_handoff_batch_advisory.py` | N/A | Allowed — scan/report |
| `rt_sa_import_dry_run.py` | N/A | Allowed — preview only |
| `rt_sa_import run-pipeline` | Forbidden | SA1 explicit commit |
| `rt_session_inspect.py` | N/A | M3 — read-only health |

| Finding ID | Verdict |
|------------|---------|
| C2-GOV-MAINT-01 | Pass |
| C2-GOV-ADV-01 | Pass — F7 batch export boundary per [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) |

---

## 8. Local-only runtime boundaries

| Check | Result |
|-------|--------|
| Loopback bridge only | **Pass** |
| Multi-session cap=3 | **Pass** |
| No distributed multi-bridge | **Pass** — forbidden in AGENTS.md and v3/v4 roadmaps |
| M3 poll does not cross machines | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| C2-GOV-LOCAL-01 | Pass |

---

## 9. Forbidden expansions (re-confirmed)

Remain forbidden without new PLAN + freeze:

- Distributed multi-bridge / cross-machine session coordination  
- SA viewer live RT hooks (beyond SA3 read-only)  
- Parser/topic/schema changes  
- Operational HITL/C2 / readiness scoring  
- Browser `capture_session`  
- Federation/orchestration authority from RT sessions  
- PX4/MAVLink/hardware assumptions  
- Post-F7 advisory expansion without new PLAN (per F7 P2 stop line)

| Finding ID | Verdict |
|------------|---------|
| C2-GOV-FORBID-01 | Pass — AGENTS.md aligned |

---

## 10. Governance verdict

**Pass — suitable for freeze.**

F7 maintainer triage and batch export v2 add **high-visibility** explanatory surfaces under existing F6 contamination controls. M3 multi-session polish does not introduce new authority paths.

**Stop line:** C2 does not authorize F8, V3, X2, or distributed runtime.
