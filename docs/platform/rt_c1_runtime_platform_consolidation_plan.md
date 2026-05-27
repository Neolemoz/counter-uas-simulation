# RT-C1 — Runtime Platform Consolidation Review (PLAN-RT-C1)

**Phase:** PLAN-RT-C1 — post-F6 platform consolidation review (docs only)  
**Prerequisite:** PLAN-RT-R2 frozen; PLAT-RT-F1 through PLAT-RT-F6 P2 frozen  
**Baseline:** [rt_r2_platform_maturity_review_r1.md](../evaluation/rt_r2_platform_maturity_review_r1.md)  
**Authority:** [AGENTS.md](../../AGENTS.md)

## Vocabulary (critical)

| Label | Meaning |
|-------|---------|
| **PLAN-RT-C1** (this wave) | Holistic **post-F6 consolidation review** — documentation only |
| **PLAT-RT-C1** | *Not used* — no implementation wave |
| **PLAN-RT-R2** / **PLAT-RT-R2d–f** | Prior maturity plateau — **not** C1 |
| **PLAN-RT-M3** | Local multi-session **polish** (background poll, inspect CLI, tab UX) per [rt_roadmap_m1_m2_v1.md](../evaluation/rt_roadmap_m1_m2_v1.md) — **not** distributed multi-bridge |
| **PLAN-RT-F7** | Post-F6 **advisory expansion** (maintainer ergonomics, contamination gates) — **not** auto-import |
| **Distributed multi-bridge** | Explicit **non-frontier** — not labeled F7 in [rt_roadmap_next_frontiers_v2.md](../evaluation/rt_roadmap_next_frontiers_v2.md) |

New artifacts use `c1_platform_*` / `rt_c1_*` filenames to avoid collision with R2 and F-wave IDs.

## Goal

Review the RT sandbox platform **as a whole** after delivery and freeze of **F1–F6** (analytics, hardening, annex review, realism, advanced experiments, fidelity coupling, SA workflow advisory) on top of the primary RT roadmap (S/G/R/T/M/TAC/SA/V/X). Produce consolidation verdict, governance re-validation, technical debt refresh, ranked next-frontier roadmap (v2), and freeze — **without** changing runtime behavior.

## Allowed

- Plan, consolidation review, governance review, technical debt audit, next-frontier roadmap v2, freeze audit
- Updates to [freeze_registry_r1.md](../evaluation/freeze_registry_r1.md), [AGENTS.md](../../AGENTS.md)
- Optional pointer in [sa_platform_maintainer_checklist.md](../evaluation/sa_platform_maintainer_checklist.md)
- Regression evidence citations from existing tests (no new tests in C1)

## Forbidden

- Changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, `src/counter_uas/`
- New bridge commands, telemetry channels, parser/topic/schema changes
- New runtime features, distributed multi-bridge, SA auto-import, tactical redesign
- Any post-C1 implementation wave without plan + governance review + freeze audit

## Review workstreams

| # | Workstream | Primary artifacts |
|---|------------|-------------------|
| 1 | Architecture consolidation | Bridge facade + handlers; F-wave modules (`experiment/`, `fidelity/`, `handoff/`); cross-wave overlaps |
| 2 | Governance | [rt_runtime_governance_v1.md](../evaluation/rt_runtime_governance_v1.md), RT↔SA, F6 advisory ≠ authority |
| 3 | Runtime consistency | Lifecycle, multi-session cap=3, tactical, capture, fidelity default-off |
| 4 | UX / cognition maturity | Workstation, F1/F3/F4/F5/F5b/F6 panels, governance banners |
| 5 | Technical debt | Hotspots, R2/F-wave closure matrix, deferred simplifications |
| 6 | Roadmap reset | v2 candidates; single advisory recommendation (M3 **or** F7) |

## Deliverables

| Artifact | Path |
|----------|------|
| Master consolidation review | [rt_c1_platform_consolidation_review_r1.md](../evaluation/rt_c1_platform_consolidation_review_r1.md) |
| Governance review | [rt_c1_platform_governance_review_r1.md](../evaluation/rt_c1_platform_governance_review_r1.md) |
| Technical debt audit | [rt_c1_technical_debt_audit_r1.md](../evaluation/rt_c1_technical_debt_audit_r1.md) |
| Next-frontier roadmap v2 | [rt_roadmap_next_frontiers_v2.md](../evaluation/rt_roadmap_next_frontiers_v2.md) |
| Freeze audit | [rt_c1_platform_consolidation_freeze_audit.md](../evaluation/rt_c1_platform_consolidation_freeze_audit.md) |

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py \
  src/counter_uas/test/test_rt_experiment_batch.py -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-rt-ui
```

Documentation-only diff hygiene: no changes under `platform/*` implementation trees or `src/counter_uas/` except this plan’s evidence citations.

## Stop line

**PLAN-RT-C1** freezes the post-F6 consolidation plateau. Do not start any new PLAT/PLAN implementation wave until:

1. An explicit wave plan addresses a ranked frontier from [rt_roadmap_next_frontiers_v2.md](../evaluation/rt_roadmap_next_frontiers_v2.md), and  
2. That wave completes governance review + freeze audit.

Recommended next frontier is **advisory only** — not authorization.

## Related

- [rt_c1_platform_consolidation_review_r1.md](../evaluation/rt_c1_platform_consolidation_review_r1.md)
- [rt_c1_platform_governance_review_r1.md](../evaluation/rt_c1_platform_governance_review_r1.md)
- [rt_c1_technical_debt_audit_r1.md](../evaluation/rt_c1_technical_debt_audit_r1.md)
- [rt_roadmap_next_frontiers_v2.md](../evaluation/rt_roadmap_next_frontiers_v2.md)
- [rt_c1_platform_consolidation_freeze_audit.md](../evaluation/rt_c1_platform_consolidation_freeze_audit.md)
