# SA H1 — Sandbox UX Architecture Freeze Audit (PLAN-SA-H1)

## Scope

This audit covers the **plan-only** H1 documentation set:

- [h1_sandbox_ux_architecture_plan.md](../platform/h1_sandbox_ux_architecture_plan.md)
- [h1_workspace_layout_notes.md](../platform/h1_workspace_layout_notes.md)

No runtime, launch, config, topic, schema, parser-contract, evaluation tooling, or `platform/sa-r0-viewer/` implementation is included in the freeze scope.

`AGENTS.md` remains the primary authority. Frozen PLAT-SA-R0–F1d audits and PLAN-SA-R1 remain authoritative for existing behavior.

## Governance Result

**Verdict: docs frozen** for PLAN-SA-H1 (plan documentation only).

The H1 plan defines sandbox workstation UX architecture, workspace tiers, five-segment navigation, panel taxonomy, declutter strategy, replay vs authoring separation, orchestration hooks (contracts only), reviewer cognition principles, and H2/H3 boundaries — without authorizing live UI, HITL semantics, operational readiness, orchestration execution, or viewer implementation.

## Boundary Checks

| Check | Result |
|-------|--------|
| Authority creep | Pass — read-only mirrors; no engagement controls |
| Parser safety | Pass — no parser-visible or schema changes proposed |
| Runtime isolation | Pass — viewer remains static JSON consumer |
| Live vs replay | Pass — replay-only; `live_observation` forbidden |
| Blended picture | Pass — explicitly forbidden; segment chrome mutually exclusive |
| Operational semantics | Pass — no C2, battle management, weapon control |
| Scoring | Pass — no readiness, certification, or ranking UX |
| Causal language | Pass — localization and divergence only |
| Legacy web/ | Pass — not extended |
| Frontend implementation | Pass — H1 docs only; H2 requires new wave |
| Orchestration execution | Pass — hooks v0 conceptual; `launch_action` forbidden |
| Architecture split | Pass — Web platform vs Gazebo runtime unchanged |

## Deliverable checklist (plan → audit)

| # | Deliverable | Plan section | Frozen |
|---|-------------|--------------|--------|
| 1 | Primary H1 architecture document | Full plan | Yes |
| 2 | Workspace layout notes | Companion doc | Yes |
| 3 | Freeze audit (this file) | — | Yes |
| 4 | Workspace tier model T0–T6 | Plan §6 | Yes |
| 5 | Five-segment navigation | Plan §8 | Yes |
| 6 | Panel taxonomy / registry IDs | Plan §9 | Yes |
| 7 | Declutter strategy | Plan §10 | Yes |
| 8 | Replay vs authoring separation | Plan §11 | Yes |
| 9 | Orchestration hooks v0 | Plan §12 | Yes |
| 10 | Cognition principles | Plan §13 | Yes |
| 11 | Governance-safe UX language | Plan §14 | Yes |
| 12 | H2/H3 boundaries | Plan §15 | Yes |
| 13 | Risks / failure modes | Plan §17 | Yes |
| 14 | Registry row PLAN-SA-H1 | freeze_registry_r1.md | Yes |

## Post-Freeze Continuation

Permitted without reopening PLAN-SA-H1:

- G1 maintenance (corpus_ref backfill, r2 snapshot, registry hygiene)
- Replay demo workflow curation
- PLAN-VIZ-R2 planning per existing doc

Requires **new scoped wave + audit** before:

- PLAT-SA-H2 viewer layout implementation
- PLAT-SA-H3 authoring/orchestration panels
- Any live ROS UI or rosbridge
- HITL or operator workflow semantics
- New `artifact_type` for layout or orchestration mirrors
- Orchestration launch from viewer

## Regression Evidence

Documentation-only wave:

- No code changes required for freeze sign-off
- Governance lint on committed SA fixtures unchanged
- `tier0-sa-r0` not required for H1 doc merge (no viewer diff)

Recommended review:

```bash
python3 scripts/evaluation/governance_lint_sa.py
```

Manual: PLAN-SA-R1 boundary checklist (see below) applied to H1 plan copy.

## Related

- [freeze_registry_r1.md](freeze_registry_r1.md) — PLAN-SA-H1 row
- [situational_awareness_ui_planning_r1.md](situational_awareness_ui_planning_r1.md) — PLAN-SA-R1 parent UX plan
- [sa_platform_frontier_review_r1.md](sa_platform_frontier_review_r1.md) — reviewer cognition P2 context
