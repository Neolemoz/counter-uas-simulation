# RT Advisory Contamination Gates (`rt_advisory_contamination_gates_v1`)

**Phase:** PLAN-RT-F7 — advisory boundary hardening (docs only)  
**Prerequisite:** [rt_f6_handoff_contamination_review_r1.md](rt_f6_handoff_contamination_review_r1.md), [rt_f6_handoff_contamination_review_p2_r1.md](rt_f6_handoff_contamination_review_p2_r1.md)  
**Authority:** [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md); [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md)

Normative **contamination gates** for post-F6 advisory expansion: RT↔SA boundary checks, lineage contamination detection, escalation limits, and forbidden automation paths.

---

## 1. Scope

Applies to:

- Per-capture derive ([rt_sa_workflow_automation_v1.md](rt_sa_workflow_automation_v1.md))
- Batch scan/report (F6 P2)
- F7 aggregation ([rt_advisory_aggregation_v1.md](rt_advisory_aggregation_v1.md))
- Future PLAT-RT-F7 UI/CLI

Does **not** change SA1 lint implementation — documents detect-only advisory signals PLAT may surface.

---

## 2. F7 contamination landmine matrix

Extends F6 F6-CONT-* with F7-CONT-*:

| ID | Landmine | Severity | PLAN mitigation | Residual (PLAT) |
|----|----------|----------|-----------------|-----------------|
| F7-CONT-01 | Queue priority triggers auto CLI | **Critical** | Queue is sort-only; contract §2 invariant | **Low** if PLAT obeys |
| F7-CONT-02 | `readiness_cohort` implies commit | **High** | Cohort labels ≠ `readiness_score`; commit gate separate | **Med** — UX copy |
| F7-CONT-03 | Batch summary authority | **Critical** | `governance_banner` required; summary cannot emit export events | **Low** |
| F7-CONT-04 | Experiment rollup overrides per-capture ladder | **High** | F6 precedence preserved; warn-only overlay | **Low** |
| F7-CONT-05 | Grouped report `--commit-all` | **Critical** | Forbidden; per-capture commit | **Med** at P2 |
| F7-CONT-06 | Lineage warn treated as auto-block commit | **Med** | `lineage` group warns; maintainer confirms lint | **Med** |
| F7-CONT-07 | Multi-session merged advisory queue | **High** | Per-capture triage; no cross-session rollup authority | **Low** |
| F7-CONT-08 | SA viewer live advisory hook | **High** | SA viewer out of scope | **Low** |
| F7-CONT-09 | Federation index from batch report | **Critical** | Forbidden | **Low** |
| F7-CONT-10 | `annotate-review` batch without gate | **High** | Gated write; not stand-up default | **Low** — F6 P2 frozen |
| F7-CONT-11 | Corpus-preview path write | **Critical** | Preview read-only; detect path targets corpus | **Low** |
| F7-CONT-12 | Escalation depth exceeds dry-run | **High** | §4 escalation table | **Med** at PLAT P2 |

---

## 3. RT↔SA boundary checks

Advisory derive and rollup **must not**:

| Check | Rule |
|-------|------|
| BND-01 | Treat SA2 `workflow_phase` as commit permission |
| BND-02 | Treat F5 `handoff_eligibility.experiment_level` as `import_ready` |
| BND-03 | Treat batch `summary.by_advisory_state` as export audit |
| BND-04 | Emit `handoff_import_committed` from derive/rollup |
| BND-05 | Write `fixtures/sa_r0/` or federation manifests |
| BND-06 | Invoke bridge `capture_session` from batch helper |

| Signal source | May inform advisory? | Authority? |
|---------------|------------------------|------------|
| `export_boundary.jsonl` | Yes (read) | No |
| `list_capture_handoff_status` mirror | Yes | No |
| `handoff_manifest.json` | Yes (read) | Prepare only via SA1 |
| `replay_sa_bundle_pack` | No (not invoked) | SA maintainer |
| Corpus `parent_ref` | After commit only | SA |

---

## 4. Lineage contamination detection

### 4.1 Detect-only signals (advisory)

When derive or rollup inspects staging manifests, emit **warnings** (not auto-fix) for:

| Signal ID | Condition | Advisory group |
|-----------|-----------|----------------|
| LIN-01 | `parent_ref` or authoritative parent equals `session_id` | `lineage` |
| LIN-02 | Conversion manifest missing `origin` containing `rt_sandbox_capture_v1` | `lineage` |
| LIN-03 | `authoritative_parent_ref` present in normalized manifest | `lineage` |
| LIN-04 | Corpus-preview targets `fixtures/sa_r0/` without subsequent explicit commit | `lineage` |
| LIN-05 | Batch report implies single-click corpus promotion | `lineage` + **forbidden** |

Detection reuses rules documented in [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md) §2–3. PLAT may add `lineage_warnings: string[]` to derive output — schema in aggregation contract.

### 4.2 What detection does not do

- Does not mutate manifests
- Does not run `rt_sa_import commit`
- Does not block derive unless `blocked` already set by F6 reject/defer/normalization rules
- Does not replace `export_boundary.py` validators

---

## 5. Escalation limits

Maximum **automation depth** without maintainer intent:

| Depth | Operation | Allowed | Stops before |
|-------|-----------|---------|--------------|
| 0 | Per-capture derive | Yes | — |
| 1 | Batch scan + filter | Yes | — |
| 2 | Grouped report + queue sort | Yes (F7) | — |
| 3 | Corpus-preview / dry-run import | Yes (default dry-run) | Writes |
| 4 | `annotate-review` gated note | Yes (explicit flags) | Bulk unattended |
| 5 | SA1 review/approve/prepare/commit | **Maintainer only** | — |

Rules:

- No chained `--execute` across steps without per-step maintainer invocation
- No promotion of advisory state to export audit events
- No elevation of `import_ready` to auto pipeline trigger
- P2 hardening may add stricter dry-run guards — requires P2 contamination re-audit (F6 pattern)

---

## 6. Forbidden automation paths

Explicit deny list (superset of F6 §4):

1. `--commit-all` or batch implicit corpus write
2. Browser subprocess: `rt_sa_import`, `rt_capture_approve`, `replay_sa_bundle_pack`
3. Bridge HTTP route for import or approve
4. Auto-import when advisory = `import_ready`
5. Auto-import when F5 experiment = `eligible`
6. Federation / orchestration publish from RT session or batch job
7. SA viewer live RT advisory subscription
8. Queue sort invoking `next_cli` automatically
9. Multi-capture failure rolling successful siblings into batch commit
10. Distributed worker executing handoff pipeline

---

## 7. Comparison to F6 reviews

| F6 control | F7 extension |
|------------|--------------|
| F6-CONT-04 batch commit | F7-CONT-05 grouped report |
| F6-CONT-09 "ready" copy | F7-CONT-02 cohort labels |
| P2 dry-run default | F7-CONT-12 escalation depth |
| Naming disambiguation | Unchanged — still required at PLAT |

Fresh **PLAT-RT-F7** waves require contamination review referencing this contract + F6 P2 review.

---

## 8. Related

- [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md)
- [rt_advisory_aggregation_v1.md](rt_advisory_aggregation_v1.md)
- [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md)
