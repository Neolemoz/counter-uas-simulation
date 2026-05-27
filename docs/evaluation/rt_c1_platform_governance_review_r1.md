# RT-C1 — Platform Governance Review R1

**Phase:** PLAN-RT-C1 — runtime platform consolidation review  
**Plan:** [rt_c1_runtime_platform_consolidation_plan.md](../platform/rt_c1_runtime_platform_consolidation_plan.md)  
**Master review:** [rt_c1_platform_consolidation_review_r1.md](rt_c1_platform_consolidation_review_r1.md)  
**Freeze audit:** [rt_c1_platform_consolidation_freeze_audit.md](rt_c1_platform_consolidation_freeze_audit.md)

## Scope verdict

| Question | Answer |
|----------|--------|
| Docs-only wave? | Yes |
| SA viewer live hooks introduced? | No — SA3 read-only replay panels only |
| Parser/topic changes? | No |
| New bridge commands from C1? | No |
| F1–F6 changed default behavior? | No new defaults claimed at consolidation freeze |
| F6 auto-import risk? | Re-checked — see §6 |

**Recommendation:** Freeze **PLAN-RT-C1** (docs frozen).

---

## 1. Authority boundaries

Re-audit against [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) and [rt_authority_model_v1.md](rt_authority_model_v1.md).

| Surface | Authoritative for sandbox? | Authoritative for SA replay? |
|---------|---------------------------|------------------------------|
| Entity registry / bridge commands | **Yes** (command truth) | No |
| Pose sync mirror | No — explanatory | No |
| Telemetry mirror / pull buffers | No — explanatory | No |
| Fidelity truth mirror (F5b) | No — truth-attested explanatory | No |
| Advisory readiness ladder (F6) | No — explanatory | No |
| Staging capture | No — pending approval | No |
| SA replay bundle | No | **Yes** (static bundle only) |

| Check | Result |
|-------|--------|
| Mirrors ≠ authority in UI | **Pass** |
| Fidelity ≠ operational sensor truth | **Pass** — `BANNER_FIDELITY_TRUTH` |
| Advisory ≠ import authority | **Pass** — `BANNER_SA_WORKFLOW_ADVISORY` |
| Experiment metrics ≠ readiness scoring | **Pass** — forbidden lexicon |

| Finding ID | Verdict |
|------------|---------|
| C1-GOV-AUTH-01 | Pass — command registry remains sole mutation authority |

---

## 2. RT↔SA separation

| Check | Result |
|-------|--------|
| `platform/sa-r0-viewer/` not used for live RT | **Pass** |
| `reject_auto_sa_import` unchanged | **Pass** |
| Manual import only (SA1) | **Pass** |
| Handoff mirror read-only (SA2) | **Pass** |
| F6 does not invoke `rt_sa_import commit` from browser | **Pass** — `isolation.test.ts` |
| `rt_sa_import_dry_run.py` preview-only | **Pass** — maintainer CLI |
| `rt_handoff_batch_advisory.py` scan/report without `--commit-all` | **Pass** — P2 freeze scope |

| Finding ID | Verdict |
|------------|---------|
| C1-GOV-SA-01 | Pass — RT writes `runs/rt_sandbox/` only |
| C1-GOV-SA-02 | Pass — capture ≠ import after F6 |
| C1-GOV-SA-03 | Pass — F6 extends cognition without shortening SA1 gates |

---

## 3. Replay boundaries

| Boundary | Enforcement |
|----------|-------------|
| RT live state ≠ replay authority | Banners, NOT SA REPLAY AUTHORITY |
| F3 annex UI ≠ SA scrubber | RT-local panels only |
| F5/F5b experiment artifacts ≠ parser contracts | Local manifests and derived reports |
| SA3 tactical continuity | Read-only counts/timeline in bundle |

| Check | Result |
|-------|--------|
| F-waves do not load SA bundles in RT UI | **Pass** |
| Replay annotations explanatory only | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| C1-GOV-REP-01 | Pass |

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

| Check | Result |
|-------|--------|
| No browser→ROS | **Pass** |
| No rosbridge / legacy web RT | **Pass** |
| No federation_register from RT UI | **Pass** |

| Finding ID | Verdict |
|------------|---------|
| C1-GOV-DENY-01 | Pass |

---

## 5. Banners and lexicon (F-wave extensions)

| Banner / label | Wave | Purpose |
|----------------|------|---------|
| `BANNER_EXPERIMENT` | X1/F1 | Compare explanatory |
| `BANNER_FIDELITY_TRUTH` | F5b | Sim-scoped truth, not ops |
| `BANNER_SA_WORKFLOW_ADVISORY` | F6 | Advisory ≠ authority |
| T1 governance chrome | T1 | Mirror non-authority |

| Check | Result |
|-------|--------|
| No unqualified “ready for operations” copy | **Pass** — fixture + test discipline |
| `import_ready` ≠ committed | **Pass** — contract + golden fixtures |

| Finding ID | Verdict |
|------------|---------|
| C1-GOV-LEX-01 | Pass |

---

## 6. F6 P2 contamination re-check

Re-validated against [rt_f6_handoff_contamination_review_r1.md](rt_f6_handoff_contamination_review_r1.md) and [rt_f6_handoff_contamination_review_p2_r1.md](rt_f6_handoff_contamination_review_p2_r1.md).

| Landmine | Post-P2 status |
|----------|----------------|
| F6-CONT-04 batch `--commit-all` | **Not present** — per-ID commit only |
| F6-CONT-05 browser subprocess import | **Not present** |
| F6-CONT-12 dry-run accidentally commits | **Mitigated** — default dry-run on P2 helpers |

| Finding ID | Verdict |
|------------|---------|
| C1-GOV-F6-01 | Pass-with-conditions — future F7 advisory expansion must repeat contamination audit |

---

## 7. Maintainer-only surfaces

| Surface | Browser | Maintainer CLI |
|---------|---------|----------------|
| `capture_session` | Forbidden | `rt_experiment_batch.py`, bridge tests |
| `rt_experiment_analytics.py` | N/A | Allowed |
| `rt_handoff_batch_advisory.py` | N/A | Allowed — scan/report |
| `rt_sa_import_dry_run.py` | N/A | Allowed — preview only |
| `rt_sa_import run-pipeline` | Forbidden | SA1 explicit commit |

| Finding ID | Verdict |
|------------|---------|
| C1-GOV-MAINT-01 | Pass |

---

## 8. Forbidden expansions (re-confirmed)

Remain forbidden without new PLAN + freeze:

- Distributed multi-bridge / cross-machine session coordination  
- SA viewer live RT hooks (beyond SA3 read-only)  
- Parser/topic/schema changes  
- Operational HITL/C2 / readiness scoring  
- Browser `capture_session`  
- Federation/orchestration authority from RT sessions  
- PX4/MAVLink/hardware assumptions  

| Finding ID | Verdict |
|------------|---------|
| C1-GOV-FORBID-01 | Pass — AGENTS.md aligned |

---

## 9. Governance verdict

**Pass — suitable for freeze.**

Advisory readiness and fidelity cognition add **high-visibility** explanatory surfaces; contamination controls from PLAN-RT-F6 remain in force. Any **PLAN-RT-F7** implementation must include a fresh contamination review before PLAT freeze.

**Stop line:** C1 does not authorize M3, F7, or distributed runtime.
