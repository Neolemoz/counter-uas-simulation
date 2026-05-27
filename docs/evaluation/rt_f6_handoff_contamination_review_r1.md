# RT-F6 — Handoff Contamination Review R1

**Phase:** PLAN-RT-F6 — auto-import and authority escalation risk (docs only)  
**Plan:** [rt_f6_sa_workflow_automation_advisory_plan.md](../platform/rt_f6_sa_workflow_automation_advisory_plan.md)  
**Contracts:** [rt_sa_workflow_automation_v1.md](rt_sa_workflow_automation_v1.md), [rt_sa_workflow_advisory_ui_v1.md](rt_sa_workflow_advisory_ui_v1.md)  
**Governance review:** [rt_f6_governance_review_r1.md](rt_f6_governance_review_r1.md)

F6 is ranked **High architecture risk / high contamination risk** in [rt_roadmap_next_frontiers_v1.md](rt_roadmap_next_frontiers_v1.md) because workflow automation "approaches auto-import if not careful." This review scores contamination vectors and PLAN mitigations.

---

## 1. Contamination landmine matrix

| ID | Landmine | Severity | PLAN mitigation | Residual risk |
|----|----------|----------|-----------------|---------------|
| F6-CONT-01 | Auto-import on `capture_session` | **Critical** | Forbidden in plan + contract; `reject_auto_sa_import()` unchanged | **Low** if PLAT obeys stop line |
| F6-CONT-02 | Auto-import when advisory reaches `import_ready` | **Critical** | `import_ready` explicitly excludes commit; no UI commit button | **Low** |
| F6-CONT-03 | F5 `handoff_eligibility: eligible` triggers import | **High** | F5 unchanged; F6 warn-only join | **Low** |
| F6-CONT-04 | Batch `--commit-all` or implicit corpus write | **Critical** | Forbidden; per-ID explicit `commit --corpus-dest` | **Med** at P2 if helpers added carelessly |
| F6-CONT-05 | Browser subprocess calling `rt_sa_import run-pipeline` | **High** | Forbidden; CLI-only P2 helpers | **Low** in PLAN |
| F6-CONT-06 | Bridge HTTP new import subcommand | **Critical** | No bridge changes in PLAN | **Low** in PLAN |
| F6-CONT-07 | SA viewer live RT hook on advisory state | **High** | SA viewer out of scope | **Low** |
| F6-CONT-08 | Federation auto-index on handoff commit | **High** | Explicitly forbidden | **Low** |
| F6-CONT-09 | Advisory state labeled "ready" without qualifier | **Med** | UI copy rules §2.1; `BANNER_SA_WORKFLOW_ADVISORY` | **Med** at PLAT P1 — UX discipline required |
| F6-CONT-10 | `readiness_score` / operational readiness creep | **Med** | Forbidden lexicon table | **Low** |
| F6-CONT-11 | Export event `handoff_ready` conflated with advisory `handoff_ready` | **Med** | Naming disambiguation + packaging-ready label | **Med** at PLAT — test with fixtures |
| F6-CONT-12 | P2 dry-run wrapper accidentally commits on success | **High** | `--dry-run` default; separate P2 governance audit | **Med** — defer P2 until P0/P1 stable |

---

## 2. Authority escalation paths (deny list)

The following must remain **impossible** after PLAN freeze and through PLAT P0/P1:

1. Bridge invokes `replay_sa_bundle_pack`
2. UI button writes to `fixtures/sa_r0/`
3. Advisory derive emits export audit events
4. Experiment batch auto-chains to `rt_sa_import commit`
5. Metrics or eligibility gates skip `rt_capture_approve.py`
6. Multi-capture failure rolls successful siblings into batch commit

PLAT P2 may add **maintainer-invoked** sequential helpers only with:

- Default `--dry-run`
- Explicit `--execute` per step
- No `--commit-all`
- Contamination re-audit before P2 freeze

---

## 3. Naming contamination review

| Confusable pair | Mitigation | PLAT test |
|-----------------|------------|-----------|
| Export `handoff_ready` vs advisory `handoff_ready` | Contract §2.1; UI "packaging ready" | Golden fixture `export_handoff_ready_pre_approve.json` |
| SA2 `workflow_phase: ready` vs advisory `handoff_ready` | Mapping table §3.1 | Fixture `sa2_ready_maps_approval_ready.json` |
| `import_ready` vs committed | Terminal state excluded | Fixture `import_ready_not_committed.json` |
| F5 experiment `eligible` vs `import_ready` | Independent badges | Fixture `f5_eligible_import_not_ready.json` |

---

## 4. Phased contamination tolerance

| PLAT phase | Allowed depth | Required gates |
|------------|---------------|----------------|
| **P0** | Read-only derive + status CLI | Architecture review findings closed; golden fixtures |
| **P1** | UI mirrors only | `tier0-rt-ui`; banner wiring; no action buttons |
| **P2** | Maintainer CLI composition | **New** governance + contamination review; default dry-run |

**Recommendation:** Authorize P0 immediately after PLAN freeze. Hold P2 until P0/P1 frozen and contamination re-check passes.

---

## 5. Comparison to frozen SA waves

| Wave | Contamination surface | F6 delta |
|------|----------------------|----------|
| PLAT-RT-SA1 | Maintainer CLIs — manual by design | F6 must not shorten SA1 gates |
| PLAT-RT-SA2 | Read-only mirror | F6 extends mirror cognitively only |
| PLAT-RT-F5 | `handoff_eligibility` advisory | F6 per-capture ladder complements — must not override |
| PLAN-RT-F6 | Automation planning | Bounded by this review |

---

## 6. Verdict

| Item | Verdict |
|------|---------|
| PLAN docs safe to freeze? | **Pass-with-conditions** |
| P0 readiness mirror? | **Authorize** — lowest contamination |
| P1 advisory UI? | **Authorize** after P0 — with UX disambiguation |
| P2 batch helpers? | **Conditional** — separate audit; default dry-run mandatory |

**Pass-with-conditions** — PLAN-RT-F6 may freeze provided PLAT phases obey §2 deny list and P2 receives a dedicated contamination re-check before implementation authorization.
