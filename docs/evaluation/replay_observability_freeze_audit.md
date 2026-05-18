# Replay Observability Freeze Audit

## Scope

This audit covers the additive replay observability tooling introduced in:

- `scripts/evaluation/replay_observability.py`
- `src/counter_uas/test/test_replay_observability.py`
- `scripts/evaluation/README.md`

No runtime, launch, schema, topic, tracker, fusion, tactical authority, or lifecycle semantic surface is included in the freeze scope.

## Governance Result

Verdict: freeze-ready.

The reviewed tooling remains evaluation-side only and emits derived, non-authoritative artifacts. The generated governance blocks distinguish raw runtime evidence, canonical parser-visible summaries, derived evaluation artifacts, and explanatory visualization layers. Reviewer-facing wording preserves the project boundaries that explanatory evidence is not authoritative state, replay logs are not parser contracts, and lifecycle/churn counters are not proof of tracker robustness.

## Boundary Checks

- Authority creep: no new authority surface was found. Reports are labeled as derived evaluation artifacts and static reports are reviewer conveniences.
- Parser safety: frozen divergence fields are consumed by name and not renamed or redefined.
- Runtime isolation: imports are limited to Python standard library and existing evaluation modules; no ROS/Gazebo runtime dependency is introduced.
- Lifecycle semantics: lifecycle/churn timelines preserve raw log line provenance and do not rename lifecycle states.
- Topology semantics: topology/timing indexing preserves `profile_id`, `scenario`, `launch_args`, geometry fields, fragmentation fields, and notes.
- Readiness claims: no operational-readiness, hardware-readiness, field P(kill), HITL/operator approval, or statistical superiority claim is introduced.

## Provenance Checks

The replay evidence bundle preserves:

- raw log and meta paths
- parser-visible summary fields
- derived evaluation fields and additive class labels
- git commit and dirty state when present
- cohort and seed lineage when present
- warnings for missing sidecar metadata or missing seed lineage

Deterministic generation was checked by generating each artifact twice from identical synthetic inputs and comparing byte output for:

- bundle JSON
- single-run report JSON
- paired comparison JSON
- topology index JSON
- governance lint JSON
- static markdown
- static HTML

Result: deterministic outputs matched.

Missing metadata behavior was checked with a log lacking sidecar metadata. The bundle emitted warnings for missing sidecar meta and missing seed lineage rather than fabricating provenance.

## Regression Evidence

Commands run:

```bash
python3 -m pytest src/counter_uas/test/test_replay_observability.py
python3 -m pytest src/counter_uas/test/test_selection_audit.py src/counter_uas/test/test_realism_metrics.py src/counter_uas/test/test_selection_oracle_divergence_taxonomy.py src/counter_uas/test/test_monte_carlo.py
python3 -m compileall -q scripts/evaluation/replay_observability.py
python3 -m pytest src/counter_uas/test
```

Results:

- replay observability focused tests: 6 passed
- related evaluation tests: 18 passed
- compile check: passed
- full `src/counter_uas/test` suite: 130 passed

## Freeze Conditions

Freeze may proceed if the commit remains limited to the audited files above and no unrelated dirty files are included.

Roadmap continuation after freeze should remain in the observability/governance UX lane. This audit does not authorize Wave 8 realism expansion, runtime redesign, parser/schema drift, or authority reinterpretation.
