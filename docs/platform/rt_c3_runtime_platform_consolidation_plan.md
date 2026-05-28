# RT-C3 — Post-X2 Platform Checkpoint Review (PLAN-RT-C3)

**Phase:** PLAN-RT-C3 — post-X2 platform consolidation review (docs only)  
**Prerequisite:** PLAN-RT-C2 frozen; PLAT-RT-V3 P0–P2 frozen; PLAT-RT-X2 P0–P2 frozen  
**Baseline:** [rt_c2_platform_consolidation_review_r1.md](../evaluation/rt_c2_platform_consolidation_review_r1.md), [rt_plat_x2_p2_freeze_audit.md](../evaluation/rt_plat_x2_p2_freeze_audit.md)  
**Authority:** [AGENTS.md](../../AGENTS.md)

## Vocabulary (critical)

| Label | Meaning |
|-------|---------|
| **PLAN-RT-C3** (this wave) | Holistic **post-X2 consolidation checkpoint** — documentation only |
| **PLAT-RT-C3** | *Not used* — no implementation wave |
| **PLAN-RT-C2** | Post-F7 consolidation plateau — **not** C3 |
| **Platform checkpoint** | Informal name from X2 P2 freeze — **same intent as C3** |
| **PLAN-RT-F8** / **V4** / **X3** | **Candidate** frontiers in v7 roadmap — **not authorized** by C3 |
| **Distributed multi-bridge** | Explicit **non-frontier** |

New artifacts use `c3_platform_*` / `rt_c3_*` filenames to avoid collision with C1, C2, and F-wave IDs.

## Goal

Review the RT sandbox platform **as a whole** after delivery and freeze of **PLAT-RT-X2** (cohort index, unified review lane, multi-manifest diff, review packet export) on top of the third consolidation plateau (C2: F7, M3, primary roadmap, F1–F6). Produce consolidation verdict, governance re-validation, technical debt refresh, ranked next-frontier roadmap (v7), and freeze — **without** changing runtime behavior.

## Allowed

- Plan, consolidation review, governance review, technical debt audit, next-frontier roadmap v7, freeze audit
- Updates to [freeze_registry_r1.md](../evaluation/freeze_registry_r1.md), [AGENTS.md](../../AGENTS.md)
- Optional pointer in [sa_platform_maintainer_checklist.md](../evaluation/sa_platform_maintainer_checklist.md)
- Regression evidence citations from existing tests (no new tests in C3)

## Forbidden

- Changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, `src/counter_uas/`
- New bridge commands, telemetry channels, parser/topic/schema changes
- New runtime features, distributed multi-bridge, SA auto-import, tactical redesign
- Authorization of F8, V4, or X3 implementation — ranking only in v7
- Any post-C3 PLAT/PLAN implementation wave without plan + governance review (+ contamination if advisory) + freeze audit

## Review workstreams

| # | Workstream | Primary artifacts |
|---|------------|-------------------|
| 1 | Architecture consolidation | Five stacks: runtime, tactical, visualization, advisory, experiment (X1/X2) |
| 2 | Governance | X2 contamination carry-forward; experiment ≠ authority; advisory ≠ authority |
| 3 | Runtime consistency | Lifecycle, cap=3, tactical, capture, fidelity default-off, F7 dry-run |
| 4 | UX / cognition maturity | V3 workstation annex; X2 workbench v2; F7 triage; M3 poll |
| 5 | Technical debt | C2/X2/V3 closure matrix; hotspots; P0/P1/residual |
| 6 | Roadmap reset | v7 candidates F8, V4, X3; single advisory PLAN recommendation |

## Deliverables

| Artifact | Path |
|----------|------|
| Master consolidation review | [rt_c3_platform_consolidation_review_r1.md](../evaluation/rt_c3_platform_consolidation_review_r1.md) |
| Governance review | [rt_c3_platform_governance_review_r1.md](../evaluation/rt_c3_platform_governance_review_r1.md) |
| Technical debt audit | [rt_c3_technical_debt_audit_r1.md](../evaluation/rt_c3_technical_debt_audit_r1.md) |
| Next-frontier roadmap v7 | [rt_roadmap_next_frontiers_v7.md](../evaluation/rt_roadmap_next_frontiers_v7.md) |
| Freeze audit | [rt_c3_platform_consolidation_freeze_audit.md](../evaluation/rt_c3_platform_consolidation_freeze_audit.md) |

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_advisory_queue.py \
  src/counter_uas/test/test_rt_handoff_batch_advisory.py -q
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

Documentation-only diff hygiene: no changes under `platform/*` implementation trees or `src/counter_uas/` except evidence citations in freeze audit.

## Stop line

**PLAN-RT-C3** freezes the post-X2 consolidation plateau. Do not start **PLAN-RT-F8**, **PLAN-RT-V4**, **PLAN-RT-X3**, or any PLAT wave until:

1. An explicit wave plan addresses a ranked frontier from [rt_roadmap_next_frontiers_v7.md](../evaluation/rt_roadmap_next_frontiers_v7.md), and  
2. That wave completes governance review (+ contamination review if advisory) + freeze audit.

Recommended next frontier is **advisory only** — not authorization.

## Related

- [rt_c3_platform_consolidation_review_r1.md](../evaluation/rt_c3_platform_consolidation_review_r1.md)
- [rt_c3_platform_governance_review_r1.md](../evaluation/rt_c3_platform_governance_review_r1.md)
- [rt_c3_technical_debt_audit_r1.md](../evaluation/rt_c3_technical_debt_audit_r1.md)
- [rt_roadmap_next_frontiers_v7.md](../evaluation/rt_roadmap_next_frontiers_v7.md)
- [rt_c3_platform_consolidation_freeze_audit.md](../evaluation/rt_c3_platform_consolidation_freeze_audit.md)
