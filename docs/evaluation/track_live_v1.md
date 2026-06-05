# TRACK-LIVE1 Live Track Workbench

Freeze ID: `PLAT-RT-TRACK-LIVE1`

Status: frozen

## Scope

TRACK-LIVE1 adds a live-backed workbench surface for selected RT sandbox entities. It renders live entity mirror information through the existing `TrackSensorWorkbenchModel` and `TrackSensorWorkbench` components while preserving the frozen TRACK-WB1 fixture path.

This layer is a live entity mirror workbench. It is not tracker output and it does not introduce live track telemetry.

## Architecture

Input flow:

```text
selectedEntityId
+ entity_pose_mirror
+ intelligence_advisory
  ↓
LiveTrackSensorAdapter
  ↓
TrackSensorWorkbenchModel
  ↓
TrackSensorWorkbench
```

The workstation attempts a live entity mirror model when live snapshots are available. If no live model can be assembled, the existing fixture path remains active.

## Live Adapter Flow

`LiveTrackSensorAdapter` maps the selected live entity into a degraded track workbench model:

- `track_id = selectedEntityId`
- `linked_entity_id = selectedEntityId`
- `track_state = live_entity_mirror`
- `track_age_s = null`
- `source_authority = entity_pose_mirror_explanatory`
- pose, heading, and speed are read from `entity_pose_mirror` when present

The adapter does not synthesize tracker output. It does not read `/tracks/state`, tracker telemetry, sensor contribution telemetry, or lifecycle telemetry.

## Provenance Model

TRACK-LIVE1 uses explicit provenance labels:

- `entity_pose_mirror_explanatory`
- `intelligence_advisory_explanatory`
- `track_live_adapter_explanatory`

The live surface displays:

- live entity mirror workbench
- not tracker output
- no live track telemetry
- entity/advisory correlation only
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
| `unknown` | Live track freshness cannot be determined from current snapshots. |

Entity mirror stale does not imply tracker coasting. Advisory stale does not imply entity mirror stale.

## Degraded-Track Strategy

Unavailable tracker fields are preserved as unavailable:

- track confidence score is `null`
- track confidence level is `unknown`
- confidence basis states that no tracker confidence, sensor fusion, or tracker lifecycle telemetry exists
- sensor contribution rows are unavailable only
- tracker lifecycle row is unavailable only
- no synthetic radar, camera, fused detection, tracker update, confirmation, coasting, or lifecycle evidence is created

## Advisory Linkage Model

Advisory linkage is sourced only from `intelligence_advisory` by matching:

```text
advisory.identity.attacker_id === selectedEntityId
```

When no live advisory is available for the selected entity, the workbench renders missing advisory state. `tactical_recommendation` is not used as an advisory replacement, confidence replacement, lifecycle replacement, or sensor contribution replacement.

## Fixture Fallback

Fixture fallback preserves the frozen TRACK-WB1 fixture behavior for:

- `track-17`
- `track-23`
- `track-31`

The selector still returns empty state for non-track fixture ids such as:

- `track-42`
- `track-55`

The live path cannot mutate fixture fixtures or fixture selectors.

## Governance Boundaries

TRACK-LIVE1 does not add or modify:

- telemetry channels
- bridge transport
- ROS topics
- parser contracts
- schemas
- assignment controls
- engagement controls
- autonomy controls

TRACK-LIVE1 does not claim tracker authority, track confidence authority, sensor fusion authority, lifecycle authority, or tactical control authority.

## Limitations

TRACK-LIVE1 has no live `/tracks/state` input. It does not provide:

- live tracker output
- live tracker id distinct from entity id
- live track confidence
- live sensor contribution evidence
- live tracker lifecycle history
- live tracker age

The live surface intentionally presents a degraded explanatory entity mirror model.

## Future Work

Future work may refine display copy or layout. Any future tracker-backed workbench requires a separately governed phase and must not be inferred from TRACK-LIVE1.

## Freeze Verdict

`PLAT-RT-TRACK-LIVE1` is frozen as a read-only live entity mirror workbench with advisory correlation and fixture fallback preserved, without bridge, ROS, telemetry channel, parser, or schema expansion.
