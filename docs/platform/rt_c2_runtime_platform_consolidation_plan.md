# RT-C2 — Runtime Platform Consolidation Review (PLAN-RT-C2)

**Phase:** PLAN-RT-C2 — post-F7 platform consolidation review (docs only)  
**Prerequisite:** PLAN-RT-C1 frozen; PLAT-RT-M3 P0–P2 frozen; PLAT-RT-F7 P0–P2 frozen  
**Baseline:** [rt_c1_platform_consolidation_review_r1.md](../evaluation/rt_c1_platform_consolidation_review_r1.md), [rt_plat_f7_p2_freeze_audit.md](../evaluation/rt_plat_f7_p2_freeze_audit.md)  
**Authority:** [AGENTS.md](../../AGENTS.md)

## Vocabulary (critical)

| Label | Meaning |
|-------|---------|
| **PLAN-RT-C2** (this wave) | Holistic **post-F7 consolidation review** — documentation only |
| **PLAT-RT-C2** | *Not used* — no implementation wave |
| **PLAN-RT-C1** | Prior post-**F6** consolidation plateau — **not** C2 |
| **Platform checkpoint** | Informal name in v3 / F7 P2 freeze — **same intent as C2** |
| **PLAN-RT-F8** / **V3** / **X2** | **Candidate** frontiers in v4 roadmap — **not authorized** by C2 |
| **Distributed multi-bridge** | Explicit **non-frontier** |

New artifacts use `c2_platform_*` / `rt_c2_*` filenames to avoid collision with C1, R2, and F-wave IDs.

## Goal

Review the RT sandbox platform **as a whole** after delivery and freeze of **PLAT-RT-F7** (advisory queue, triage UI, batch export v2) and **PLAT-RT-M3** (session inspect, poll UX, tab reorder) on top of the primary RT roadmap and F1–F6 plateau documented at C1. Produce consolidation verdict, governance re-validation, technical debt refresh, ranked next-frontier roadmap (v4), and freeze — **without** changing runtime behavior.

## Allowed

- Plan, consolidation review, governance review, technical debt audit, next-frontier roadmap v4, freeze audit
- Updates to [freeze_registry_r1.md](../evaluation/freeze_registry_r1.md), [AGENTS.md](../../AGENTS.md)
- Optional pointer in [sa_platform_maintainer_checklist.md](../evaluation/sa_platform_maintainer_checklist.md)
- Regression evidence citations from existing tests (no new tests in C2)

## Forbidden

- Changes under `platform/rt-sandbox-bridge/`, `platform/rt-sandbox-ui/`, `platform/sa-r0-viewer/`, `src/counter_uas/`
- New bridge commands, telemetry channels, parser/topic/schema changes
- New runtime features, distributed multi-bridge, SA auto-import, tactical redesign
- Authorization of F8, V3, or X2 implementation — ranking only in v4
- Any post-C2 PLAT/PLAN implementation wave without plan + governance review + freeze audit

## Review workstreams

| # | Workstream | Primary artifacts |
|---|------------|-------------------|
| 1 | Architecture consolidation | Bridge + handlers; F6/F7 advisory; M3 multi-session; experiment/fidelity stacks |
| 2 | Governance | F7 contamination re-check; advisory ≠ authority; RT↔SA separation |
| 3 | Runtime consistency | Lifecycle, cap=3, tactical, capture, fidelity default-off, F7 dry-run |
| 4 | UX / cognition maturity | F7 triage queue, M3 poll/tab UX, workstation density |
| 5 | Technical debt | C1/M3/F7 closure matrix; hotspots; P0/P1/residual |
| 6 | Roadmap reset | v4 candidates F8, V3, X2; single advisory recommendation |

## Deliverables

| Artifact | Path |
|----------|------|
| Master consolidation review | [rt_c2_platform_consolidation_review_r1.md](../evaluation/rt_c2_platform_consolidation_review_r1.md) |
| Governance review | [rt_c2_platform_governance_review_r1.md](../evaluation/rt_c2_platform_governance_review_r1.md) |
| Technical debt audit | [rt_c2_technical_debt_audit_r1.md](../evaluation/rt_c2_technical_debt_audit_r1.md) |
| Next-frontier roadmap v4 | [rt_roadmap_next_frontiers_v4.md](../evaluation/rt_roadmap_next_frontiers_v4.md) |
| Freeze audit | [rt_c2_platform_consolidation_freeze_audit.md](../evaluation/rt_c2_platform_consolidation_freeze_audit.md) |

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

**PLAN-RT-C2** freezes the post-F7 consolidation plateau. Do not start **PLAN-RT-F8**, **PLAN-RT-V3**, **PLAN-RT-X2**, or any PLAT wave until:

1. An explicit wave plan addresses a ranked frontier from [rt_roadmap_next_frontiers_v4.md](../evaluation/rt_roadmap_next_frontiers_v4.md), and  
2. That wave completes governance review (+ contamination review if advisory) + freeze audit.

Recommended next frontier is **advisory only** — not authorization.

## Related

- [rt_c2_platform_consolidation_review_r1.md](../evaluation/rt_c2_platform_consolidation_review_r1.md)
- [rt_c2_platform_governance_review_r1.md](../evaluation/rt_c2_platform_governance_review_r1.md)
- [rt_c2_technical_debt_audit_r1.md](../evaluation/rt_c2_technical_debt_audit_r1.md)
- [rt_roadmap_next_frontiers_v4.md](../evaluation/rt_roadmap_next_frontiers_v4.md)
- [rt_c2_platform_consolidation_freeze_audit.md](../evaluation/rt_c2_platform_consolidation_freeze_audit.md)
