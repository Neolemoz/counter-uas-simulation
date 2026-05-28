# RT Advisory Contamination Gates v2 (`rt_advisory_contamination_gates_v2`)

**Phase:** PLAN-RT-F8 — advisory boundary hardening (docs only)  
**Prerequisite:** [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) (F7 frozen); [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md); PLAT-RT-F7 P0–P2 frozen  
**Authority:** [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md); [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md); [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)

Normative **contamination gates** for post-F7 advisory expansion: filter presets, aggregation v2, X2 review-packet adjacency, corpus-preview refinements, escalation limits v2, and forbidden automation depth.

**Highest governance sensitivity** in the F8 wave — all other F8 contracts must comply with this document.

---

## 1. Scope

Applies to:

- F7-delivered derive, batch summary v1, batch review v2, triage UI
- F8 aggregation v2 ([rt_advisory_aggregation_v2.md](rt_advisory_aggregation_v2.md))
- F8 maintainer presets and templates ([rt_advisory_maintainer_workflow_v2.md](rt_advisory_maintainer_workflow_v2.md))
- X2 experiment cohort index + review packet export (read-only adjacency only)
- Future PLAT-RT-F8 UI/CLI

Does **not** change SA1 lint implementation — documents detect-only advisory signals PLAT may surface.

---

## 2. F8 contamination landmine matrix

Extends F7-CONT-01..12 with F8-CONT-*:

| ID | Landmine | Severity | PLAN mitigation | Residual (PLAT) |
|----|----------|----------|-----------------|-----------------|
| F8-CONT-01 | Filter preset triggers batch approve | **Critical** | Presets are filter/sort only; no CLI invocation | **Low** if PLAT obeys |
| F8-CONT-02 | `readiness_cohort_v2` implies commit | **High** | v2 buckets ≠ `readiness_score`; commit gate separate | **Med** — UX copy |
| F8-CONT-03 | Aggregation v2 summary as authority | **Critical** | `governance_banner` required; v2 cannot emit export events | **Low** |
| F8-CONT-04 | Experiment/handoff rollup overrides ladder | **High** | F6 precedence; warn-only overlay | **Low** |
| F8-CONT-05 | Stand-up template embeds auto `next_cli` run | **Critical** | Templates are render-only shells | **Med** at P2 |
| F8-CONT-06 | X2 review packet implies corpus commit | **High** | Packet export ≠ staging authority; banner required | **Med** — combined UX |
| F8-CONT-07 | Multi-capture cohort summary writes corpus | **Critical** | Rollup read-only; no `fixtures/sa_r0/` write | **Low** |
| F8-CONT-08 | Focus set merges cross-session authority | **High** | Focus = ID list for cognition; per-capture triage | **Low** |
| F8-CONT-09 | M3 background poll merges advisory queue | **High** | Poll mirrors telemetry only; no merged queue authority | **Low** |
| F8-CONT-10 | Cohort v2 label skips approve step | **High** | Labels map states; SA1 CLIs unchanged | **Med** |
| F8-CONT-11 | Corpus-preview refinement path write | **Critical** | Preview read-only; explicit commit only | **Low** — F6 P2 pattern |
| F8-CONT-12 | Template render exceeds dry-run depth | **High** | §5 escalation v2 — max depth 3 before commit | **Med** at P2 |
| F8-CONT-13 | X2 experiment cohort index as import gate | **High** | Cohort index ≠ readiness cohort; warn-only | **Med** |
| F8-CONT-14 | Batch v2 exemplar IDs imply promotion | **Med** | Exemplars capped; full list in `captures[]` only | **Low** |

F7-CONT-01..12 remain in force — F8 does not relax F7 mitigations.

---

## 3. RT↔SA boundary checks

Carry forward BND-01..06 from [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) §3.

F8 additions:

| Check | Rule |
|-------|------|
| BND-07 | Treat X2 `review_packet` file export as explanatory — not `handoff_manifest.json` authority |
| BND-08 | Treat X2 `multi_manifest_diff` as metadata cognition — not approve/prepare permission |
| BND-09 | Treat F8 `experiment_handoff_rollup` as warn-only — not `import_ready` |
| BND-10 | Treat filter preset `import_advisory_only` as row filter — not auto `rt_sa_import commit` |
| BND-11 | Treat stand-up template Markdown/JSON shell as copy artifact — not pipeline trigger |

| Signal source | May inform advisory? | Authority? |
|---------------|------------------------|------------|
| X2 cohort index (`rt_experiment_cohort_index_v1`) | Yes (read) | No |
| X2 review packet export path | Yes (read) | No |
| F8 `summary.handoff_rollup` | Yes (read) | No |
| F8 template pack output file | Yes (maintainer opt-in write) | No — not corpus commit |

---

## 4. Lineage contamination detection v2

### 4.1 Carry forward LIN-01..05

Per [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md) §4 — unchanged semantics.

### 4.2 F8 extensions (detect-only)

| Signal ID | Condition | Advisory group |
|-----------|-----------|----------------|
| LIN-06 | Batch v2 `grouped` exemplar list used as sole commit checklist | `lineage` + **forbidden** automation |
| LIN-07 | `parent_ref` surfaced only in rollup summary without per-capture lint | `lineage` — warn |
| LIN-08 | Corpus-preview refinement suggests default `fixtures/sa_r0/` dest | `lineage` + **forbidden** implicit write |
| LIN-09 | X2 packet `manifest_ref` treated as authoritative parent for SA corpus | `lineage` + **forbidden** |
| LIN-10 | Multi-capture cohort summary implies single-click promotion | `lineage` + **forbidden** |

### 4.3 What detection does not do

Unchanged from F7 §4.2 — no manifest mutation, no auto `commit`, no replacement of `export_boundary.py` validators.

---

## 5. Escalation limits v2

Maximum **automation depth** without maintainer intent:

| Depth | Operation | Allowed | Stops before |
|-------|-----------|---------|--------------|
| 0 | Per-capture derive | Yes | — |
| 1 | Batch scan + filter / preset | Yes | — |
| 2 | Summary v1/v2 + queue sort + grouped indexes | Yes | — |
| 3 | Stand-up template render + corpus-preview / dry-run | Yes (default dry-run) | Corpus writes |
| 4 | `annotate-review` gated note | Yes (explicit flags) | Bulk unattended |
| 5 | SA1 review/approve/prepare/commit | **Maintainer only** | — |

F8 rules:

- Filter preset application is **depth 1** — must not chain to depth 5
- Template pack output at depth 3 requires `dry_run: true` on enclosing batch document
- No elevation of `import_ready` or `ready_for_commit_advisory` to pipeline trigger
- X2 review packet generation from batch helper is **forbidden** — separate maintainer action only

---

## 6. Forbidden automation paths

Explicit deny list — superset of F7 §6:

1. `--commit-all` or batch implicit corpus write
2. Preset or focus set invoking `next_cli` subprocess
3. Stand-up template with embedded `--execute` or auto pipeline flags
4. Browser subprocess: `rt_sa_import`, `rt_capture_approve`, `replay_sa_bundle_pack`
5. Bridge HTTP route for import or approve
6. Auto-import when advisory = `import_ready` or cohort v2 = `ready_for_commit_advisory`
7. Auto-import when F5 experiment = `eligible` or X2 cohort = `complete`
8. Federation / orchestration publish from RT session or batch job
9. SA viewer live RT advisory subscription or rollup consumption
10. Queue sort or preset triggering SA1 CLIs
11. Multi-capture failure rolling successful siblings into batch commit
12. Distributed worker executing handoff pipeline
13. Batch helper generating X2 review packet as authority artifact

---

## 7. PLAT phase tolerance

| PLAT phase | Allowed depth | Required gates |
|------------|---------------|--------------|
| **P0** | 0–2 | F8 contamination review closed for P0; golden fixtures |
| **P1** | 0–2 UI mirror | Read-only panels; no action buttons; banner reuse |
| **P2** | 0–3 | P2 contamination re-audit; template + preview refinements audited |

Fresh **PLAT-RT-F8** waves require contamination review referencing this contract + [rt_f7_handoff_contamination_review_r1.md](rt_f7_handoff_contamination_review_r1.md) + F6 P2 review.

---

## 8. Comparison to F7 gates

| F7 control | F8 extension |
|------------|--------------|
| F7-CONT-05 grouped report | F8-CONT-05 template auto-run |
| F7-CONT-02 cohort labels | F8-CONT-02 cohort v2 + F8-CONT-10 skip approve |
| F7-CONT-12 escalation | F8-CONT-12 template depth |
| — | F8-CONT-06 X2 packet + F8-CONT-13 cohort index |

---

## 9. Related

- [rt_advisory_maintainer_workflow_v2.md](rt_advisory_maintainer_workflow_v2.md)
- [rt_advisory_aggregation_v2.md](rt_advisory_aggregation_v2.md)
- [rt_f8_handoff_contamination_review_r1.md](rt_f8_handoff_contamination_review_r1.md)
- [rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md)
- [rt_experiment_unified_review_v1.md](rt_experiment_unified_review_v1.md)
