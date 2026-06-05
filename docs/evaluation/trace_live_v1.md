# TRACE-LIVE1 Live Traceability Layer

Freeze ID: `PLAT-RT-TRACE-LIVE1`

Status: frozen

## Scope

TRACE-LIVE1 adds a live-backed traceability view for the RT sandbox workstation. It correlates a selected live entity with the existing intelligence advisory transport and renders the result through the existing traceability workbench model.

This layer is live entity/advisory correlation only. It is not tracker lineage.

## Architecture

Input flow:

```text
selectedEntityId
+ entity_pose_mirror
+ intelligence_advisory
  ↓
LiveTraceabilityAdapter
  ↓
TraceabilityAssemblyInput
  ↓
assembleTraceabilityWorkbenchModel
  ↓
TraceabilityWorkbenchModel
  ↓
TraceabilityWorkbench
```

The fixture-backed TRACE1 path remains available and unchanged. The workstation attempts live traceability first when live snapshots are present, then falls back to the existing fixture path when no live model can be assembled.

## Live Adapter Flow

`LiveTraceabilityAdapter` converts the selected live entity into a degraded track-like assembly input:

- `track_id = selectedEntityId`
- `linked_entity_id = selectedEntityId`
- `track_state = live_entity_mirror`
- `source_authority = entity_pose_mirror_explanatory`
- confidence score is `null`
- confidence level is `unknown`
- sensor contribution rows are empty
- lifecycle rows are empty

The adapter does not synthesize tracker confidence, sensor fusion evidence, or tracker lifecycle events.

The advisory side is sourced only from `intelligence_advisory` by matching:

```text
advisory.identity.attacker_id === selectedEntityId
```

If no advisory is available for the selected live entity, the workbench renders missing advisory state and does not substitute tactical recommendation data.

## Provenance Model

TRACE-LIVE1 uses explicit provenance labels:

- `entity_pose_mirror_explanatory`
- `intelligence_advisory_explanatory`
- `traceability_live_adapter_explanatory`

The live surface displays:

- live entity/advisory correlation
- not tracker lineage
- recommendation origin visibility only
- read-only explanation surface
- no assignment authority
- no engagement authority
- no autonomy authority

## Freshness Model

Freshness is derived only from entity mirror freshness and intelligence advisory stale metadata:

| Freshness | Meaning |
|-----------|---------|
| `fresh` | Entity mirror fresh; advisory fresh. |
| `entity_stale` | Entity mirror stale; advisory is not marked stale. |
| `advisory_stale` | Entity mirror fresh; advisory is marked stale. |
| `both_stale` | Entity mirror stale; advisory is marked stale. |
| `unknown` | Live freshness cannot be determined from current snapshots. |

TRACE-LIVE1 does not infer tracker lifecycle staleness.

## Fixture Fallback

Fixture fallback preserves the frozen TRACE1 fixture behavior for:

- `track-17`
- `track-23`
- `track-31`
- `track-42`
- `track-55`

The fixture path remains backed by `TRACEABILITY_FIXTURE_INPUTS`, `SelectedTrackTraceabilityWorkbench`, and existing traceability selectors.

## Governance Boundaries

TRACE-LIVE1 does not add or modify:

- telemetry channels
- bridge transport
- ROS topics
- parser contracts
- schemas
- tactical assignment controls
- engagement controls
- autonomy controls

`tactical_recommendation` is not used as an advisory replacement, advisory fallback, or advisory synthesis source.

## Limitations

TRACE-LIVE1 has no live `/tracks/state` input. It does not provide:

- live tracker lineage
- live track id distinct from entity id
- live sensor contribution evidence
- live tracker lifecycle records
- live tracker confidence

The live surface intentionally presents a degraded explanatory correlation model.

## Future Work

Future work may refine UI copy or layout for the live traceability surface. Any future live tracker lineage work would require a separately governed phase and must not be inferred from TRACE-LIVE1.

## Freeze Verdict

`PLAT-RT-TRACE-LIVE1` is frozen as a read-only live entity/advisory traceability layer with fixture fallback preserved and no bridge, ROS, telemetry channel, parser, or schema expansion.
