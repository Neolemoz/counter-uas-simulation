# Replay Narrative UX Freeze Audit

## Scope

This audit covers the additive Phase 2 Replay Narrative UX implementation:

- [`replay_narrative_ux_phase2_plan.md`](replay_narrative_ux_phase2_plan.md)
- [`scripts/evaluation/replay_observability.py`](../../scripts/evaluation/replay_observability.py)
- [`src/counter_uas/test/test_replay_observability.py`](../../src/counter_uas/test/test_replay_observability.py)
- [`scripts/evaluation/README.md`](../../scripts/evaluation/README.md)

`AGENTS.md` remains the primary governance authority. No runtime, launch, topic, schema, parser-contract, tracker, fusion, tactical authority, HITL/operator, readiness, certification, or live dashboard surface is included in this freeze scope.

## Governance Result

Verdict: freeze-ready for the scoped replay narrative UX layer.

The implementation adds `replay_narrative_v1` as a derived evaluation artifact generated from existing replay observability reports. It groups lifecycle, selection, divergence, ambiguity, outcome, and provenance-warning evidence into deterministic static narratives. The artifact is explicitly non-authoritative and does not replace parser-visible summaries, runtime topics, tactical authority surfaces, lifecycle semantics, or replay contracts.

## Boundary Checks

- Authority creep: no new authority surface was introduced. Narrative reports are labeled as derived evaluation artifacts and reject unified authoritative replay-state interpretation.
- Parser safety: no parser-visible fields were renamed, removed, or reinterpreted. Parser summaries remain copied context only.
- Runtime isolation: implementation stays in evaluation-side Python and uses existing replay observability inputs; no ROS/Gazebo runtime dependency, websocket, or live dashboard coupling was introduced.
- Lifecycle semantics: lifecycle events remain explanatory overlays over raw log lines and are not tracker lifecycle truth or robustness proof.
- Divergence semantics: selection/oracle mismatch windows localize replay-side evidence only and are explicitly non-causal.
- Provenance: missing metadata and visibility-limited conditions remain warnings or interpretation prompts, not severity, readiness, or approval status.
- Readiness claims: no operational-readiness, hardware-readiness, certification, field performance, HITL/operator approval, tactical command, or composite robustness score was introduced.

## Artifact Contract

The new narrative artifact has:

- `artifact_type: replay_narrative_report`
- `narrative_schema_version: replay_narrative_v1`
- existing replay observability governance block
- copied lineage and source artifact references
- deterministic `events` with category, event type, label, source reference, source artifact, evidence role, and interpretation caveat
- deterministic `windows` for supported fragmentation and divergence contexts
- summary counts and parser-visible summary mirrors
- visible warnings and interpretation caveats

This schema is additive and derived. It is not a parser contract and downstream runtime or aggregate evaluation tools are not required to consume it.

## Determinism Checks

Focused tests verify that:

- replay narrative generation is deterministic for identical inputs;
- static Markdown rendering is deterministic for identical narrative JSON;
- static HTML rendering is deterministic for identical Markdown;
- governance lint accepts the narrative artifact only when schema and anti-claim caveats are present.

## Regression Evidence

Commands run:

```bash
python3 -m pytest src/counter_uas/test/test_replay_observability.py
python3 -m compileall -q scripts/evaluation/replay_observability.py
```

Results:

- replay observability focused tests: 11 passed
- compile check: passed

## Freeze Conditions

Freeze may proceed if the change set remains limited to documentation, evaluation-side replay observability tooling, and focused tests. Post-freeze continuation should remain in the static replay narrative UX lane.

This audit does not authorize runtime redesign, parser/schema/topic changes, tracker/fusion redesign, live dashboard coupling, operator/HITL semantics, readiness scoring, AI causal explanation systems, governance automation expansion, or realism/runtime capability expansion.
