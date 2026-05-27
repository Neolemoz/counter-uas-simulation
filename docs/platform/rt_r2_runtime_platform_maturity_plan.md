# RT-R2 — Runtime Platform Maturity Review (PLAN-RT-R2)

**Phase:** PLAN-RT-R2 — post-roadmap platform maturity review (docs only)  
**Prerequisite:** Primary RT interactive sandbox roadmap complete through **PLAT-RT-X1** (frozen)  
**Baseline:** **PLAN-RT-R1** ([rt_r1_architecture_stabilization_review_r1.md](../evaluation/rt_r1_architecture_stabilization_review_r1.md))  
**Authority:** [AGENTS.md](../../AGENTS.md)

## Vocabulary (critical)

| Label | Meaning |
|-------|---------|
| **PLAN-RT-R2** (this wave) | Holistic **platform maturity review** — documentation only |
| **PLAT-RT-R2d / R2e / R2f** (frozen) | R1-era **closure implementation waves** — not this review |

New artifacts use `r2_platform_*` filenames to avoid collision with [rt_r2d_governance_review_r1.md](../evaluation/rt_r2d_governance_review_r1.md).

## Goal

Review the RT sandbox platform **as a whole** after end-to-end delivery (S1–S6, G1–G6, R1 maintenance, T1–T5, M1–M2, SA1–SA3, V1–V2, TAC1–TAC5, X1). Produce maturity verdict, governance re-validation, technical debt inventory, ranked next-frontier roadmap, and freeze — **without** changing runtime behavior.

## Allowed

- Plan, maturity review, governance review, technical debt audit, next-frontier roadmap, freeze audit
- Updates to [freeze_registry_r1.md](../evaluation/freeze_registry_r1.md), [AGENTS.md](../../AGENTS.md)
- Optional pointer in [sa_platform_maintainer_checklist.md](../evaluation/sa_platform_maintainer_checklist.md)
- Regression evidence citations from existing tests (no new tests in R2)

## Forbidden

- Changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, `src/counter_uas/`
- New bridge commands, telemetry channels, parser/topic/schema changes
- New runtime features, federation automation, distributed multi-bridge, SA auto-import
- Any post-R2 implementation wave without plan + governance review + freeze audit

## Review workstreams

| # | Workstream | Primary artifacts |
|---|------------|-------------------|
| 1 | Architecture | Bridge facade + handlers, adapter/G6, RT UI, tactical stack, X1 experiment |
| 2 | Governance | [rt_runtime_governance_v1.md](../evaluation/rt_runtime_governance_v1.md), RT↔SA, replay, deny-by-default |
| 3 | Runtime consistency | Lifecycle, tactical, telemetry, capture continuity, multi-session |
| 4 | UX maturity | Workstation, Cesium, experimentation workflow |
| 5 | Technical debt | Hotspots, duplication, coverage, R1 residual/closed matrix |
| 6 | Next frontiers | Ranked candidates; single recommended major frontier |

## Deliverables

| Artifact | Path |
|----------|------|
| Master maturity review | [rt_r2_platform_maturity_review_r1.md](../evaluation/rt_r2_platform_maturity_review_r1.md) |
| Governance review | [rt_r2_platform_governance_review_r1.md](../evaluation/rt_r2_platform_governance_review_r1.md) |
| Technical debt audit | [rt_r2_technical_debt_audit_r1.md](../evaluation/rt_r2_technical_debt_audit_r1.md) |
| Next-frontier roadmap | [rt_roadmap_next_frontiers_v1.md](../evaluation/rt_roadmap_next_frontiers_v1.md) |
| Freeze audit | [rt_r2_platform_maturity_freeze_audit.md](../evaluation/rt_r2_platform_maturity_freeze_audit.md) |

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

**PLAN-RT-R2** freezes the maturity plateau. Do not start any new PLAT/PLAN implementation wave until:

1. An explicit wave plan addresses a ranked frontier from [rt_roadmap_next_frontiers_v1.md](../evaluation/rt_roadmap_next_frontiers_v1.md), and  
2. That wave completes governance review + freeze audit.

Recommended next frontier is **advisory only** — not authorization.

## Related

- [rt_r2_platform_maturity_review_r1.md](../evaluation/rt_r2_platform_maturity_review_r1.md)
- [rt_r2_platform_governance_review_r1.md](../evaluation/rt_r2_platform_governance_review_r1.md)
- [rt_r2_technical_debt_audit_r1.md](../evaluation/rt_r2_technical_debt_audit_r1.md)
- [rt_roadmap_next_frontiers_v1.md](../evaluation/rt_roadmap_next_frontiers_v1.md)
- [rt_r2_platform_maturity_freeze_audit.md](../evaluation/rt_r2_platform_maturity_freeze_audit.md)
