# RT-F6 — Handoff Contamination Review P2 R1

**Phase:** PLAT-RT-F6 P2 — batch maintainer helpers  
**Prior review:** [rt_f6_handoff_contamination_review_r1.md](rt_f6_handoff_contamination_review_r1.md)  
**Plan:** [rt_plat_f6_p2_batch_maintainer_helpers_implementation_plan.md](../platform/rt_plat_f6_p2_batch_maintainer_helpers_implementation_plan.md)

Re-check of P2 CLI surface against F6 contamination landmines.

---

## P2 landmine re-score

| ID | Landmine | P2 mitigation | Residual |
|----|----------|---------------|----------|
| F6-CONT-01 | Auto-import on capture | No import in batch/dry-run CLIs | **Low** |
| F6-CONT-02 | Auto-import at import_ready | Advisory hints only; no commit | **Low** |
| F6-CONT-04 | Batch `--commit-all` | Flag absent; tests assert | **Low** |
| F6-CONT-05 | Browser subprocess | CLI-only | **Low** |
| F6-CONT-12 | Dry-run wrapper commits | `rt_sa_import_dry_run` never calls commit | **Low** |

---

## Deny list (P2)

1. No `commit_bundle_to_corpus` without explicit `rt_sa_import commit`  
2. No `--commit-all` / `--auto-import`  
3. `annotate-review` requires `--allow-write` + `--confirm-capture-id` + `--no-dry-run`  
4. `reject_auto_sa_import` unchanged  

---

## Verdict

**Pass** — P2 surface satisfies contamination gates with default dry-run and per-capture isolation.
