# Track → Threat Traceability Layer V1

Freeze ID: `PLAT-RT-TRACE1`

Status: frozen

## Scope

Track → Threat Traceability Layer V1 is a read-only RT sandbox workstation surface
for inspecting fixture-backed lineage from track through threat evaluation to
advisory recommendation origin. It gives reviewers a bounded place to inspect
linkage status, freshness alignment, and recommendation provenance without
changing telemetry, bridge transport, ROS topics, schemas, tactical controls,
assignment behavior, engagement behavior, autonomy, or SA/replay tooling.

This freeze covers:

- `TraceabilityWorkbench`
- `TraceabilitySummaryPanel`
- `TrackLineagePanel`
- `ThreatLineagePanel`
- `AdvisoryOriginPanel`
- `traceabilitySelectors.ts` (linkage resolution, freshness alignment, assembly)
- `getSelectedTraceabilityModel()`
- `SelectedTrackTraceabilityWorkbench`
- `TrackTraceabilityWorkstationSurface` workstation placement
- fixture-backed selected-track flow
- empty selected-track state
- stale, partial, mismatch, and missing-advisory visibility
- component, selector, integration, and workstation placement tests

## Architecture

The workbench is implemented under:

- `platform/rt-sandbox-ui/src/traceability/workbench/`
- `platform/rt-sandbox-ui/src/workstation/AppWorkstationSlots.tsx`

The workstation-level placement is:

`AppWorkstationSlots -> TrackTraceabilityWorkstationSurface -> SelectedTrackTraceabilityWorkbench -> TraceabilityWorkbench`

The surface is separate from tactical and intelligence command panels. It is
rendered in the workstation tactical column immediately after the Track & Sensor
Workbench surface. The workbench exposes no buttons, callbacks, assignment
controls, engagement controls, approve/reject controls, or autonomous controls.

## Lineage Chain

V1 models the read-only explanation chain:

`Track -> Attacker -> Threat Evaluation -> Advisory -> Recommended Defender`

Primary linkage match:

- `linked_entity_id` ↔ `attacker_id`

Assembly inputs:

- `TrackSensorWorkbenchModel` (local fixture track mirror)
- `RtIntelligenceAdvisoryV1` (local fixture advisory object)
- optional local fixture metadata (`advisory_stale`, `advisory_stale_reason`)

There is no telemetry subscription, no bridge request, no ROS topic read, no
schema change, and no SA coupling in V1.

## Selected-Track Flow

V1 is selected-track driven and fixture backed:

1. `TrackTraceabilityWorkstationSurface` receives the current selected ID as
   `selectedTrackId` from the existing workstation selection flow
   (`selectedEntityId` in `AppWorkstationSlots`).
2. `SelectedTrackTraceabilityWorkbench` calls `getSelectedTraceabilityModel`.
3. The selector resolves that ID against `TRACEABILITY_FIXTURE_INPUTS`.
4. `assembleTraceabilityWorkbenchModel` builds the full lineage model.
5. Matching fixture records render the full workbench.
6. Missing or empty selections render the empty state.

There is no automatic track selection, no persistence, and no generated
placeholder lineage.

## Workstation Placement

The workstation surface is mounted inside the connected workstation tactical
column in `AppWorkstationSlots.tsx`, after `TrackSensorWorkbenchWorkstationSurface`
and before `TacticalManualPanel`. It is intentionally independent from:

- `IntelligenceAdvisoryWorkstationSurfaces`
- `ThreatEvaluationWorkbench`
- `TacticalManualPanel`
- `TacticalAssistedPanel`
- `TacticalAutonomousPanel`
- SA replay viewer tooling

The surface can render:

- fully linked fixture lineage
- partial fixture lineage
- missing-advisory fixture lineage
- stale fixture lineage
- mismatch fixture lineage
- empty state when the selected ID has no fixture-backed traceability input

## Traceability Summary

`TraceabilitySummaryPanel` displays:

- `track_id`
- `attacker_id`
- `threat_rank`
- `threat_score`
- `advisory_id`
- `recommended_defender`
- `linkage_status`
- `freshness_alignment`

`linkage_status` is rendered with read-only tone chips for linked, partial,
missing, stale, and mismatch states.

## Track Lineage

`TrackLineagePanel` displays the chain:

`Track -> Entity -> Attacker`

Fields:

- `track_id`
- `linked_entity_id`
- `attacker_id`
- `track_state`
- `track_age`
- `freshness`
- `confidence_level`
- `confidence_score`
- `confidence_basis`

Values come from the selected fixture track model and assembled linkage
resolution. They are not inferred from live telemetry.

## Threat Lineage

`ThreatLineagePanel` displays the chain:

`Attacker -> Threat Evaluation`

Fields:

- `attacker_id`
- `threat_rank`
- `threat_score`
- `threat_components`
- `heuristic_confidence_level`
- `heuristic_confidence_score`
- `confidence_basis`

When threat evaluation is unavailable, the panel displays a missing-state
message instead of hiding the section.

Threat lineage confidence is heuristic and explanatory only. It is not mission
success confidence or kill probability.

## Advisory Origin

`AdvisoryOriginPanel` displays the chain:

`Threat Evaluation -> Advisory`

Fields:

- `advisory_id`
- `attacker_id`
- `recommended_defender`
- `defender_rank`
- `tti_s`
- `reason_codes`
- `explanation`
- `advisory_freshness`
- `advisory_utc`
- `stale_reason`

Supported states:

- no recommendation
- stale advisory
- partial linkage

When advisory origin is unavailable, the panel displays a missing-state message
instead of hiding the section.

## Linkage Status Model

`resolveTraceabilityLinkage` derives read-only status from fixture inputs:

| Status | Meaning |
|--------|---------|
| `linked` | `linked_entity_id` matches advisory attacker; advisory link present; not stale |
| `partial` | Threat/advisory context exists but advisory link is incomplete |
| `missing` | No advisory and no advisory link for the selected track |
| `stale` | Track stale and/or advisory marked stale in local fixture metadata |
| `mismatch` | `linked_entity_id` does not match advisory `attacker_id` |

No schema changes are introduced for linkage status. Status is UI-local only.

## Freshness Alignment Model

`deriveFreshnessAlignment` derives read-only alignment tokens from local fixture
metadata and track/advisory freshness:

| Token | Meaning |
|-------|---------|
| `track_and_advisory_fresh` | Linked track and advisory both fresh |
| `track_fresh_advisory_missing` | Track fresh; advisory record not linked |
| `advisory_unavailable` | No advisory path available |
| `track_stale_advisory_preserved` | Track stale; advisory preserved for review |
| `attacker_id_mismatch` | Track and advisory attacker identities diverged |
| `both_stale` | Track and advisory both stale |
| `track_stale` | Track stale only |
| `advisory_stale` | Advisory stale only |
| `unknown` | Alignment cannot be determined from fixture inputs |

When `TrackAdvisoryLink.freshness_alignment` is present on the fixture track
model, stale/linked scenarios may preserve that token for workstation continuity.

## Stale Behavior

Stale lineage remains visible. When track or advisory freshness is stale, the
workbench:

- displays a stale banner
- keeps the compact context header visible
- keeps track lineage visible
- keeps threat lineage visible when available
- keeps advisory origin visible when available

Stale advisory origin may include `stale_reason` from local fixture metadata.
Stale data remains historical/explanatory only.

## Mismatch Behavior

When `linked_entity_id` and advisory `attacker_id` diverge:

- summary `linkage_status` is `mismatch`
- a mismatch banner is shown
- track lineage uses the track-linked attacker identity
- threat lineage uses the track-linked attacker identity with advisory threat data
- advisory origin preserves the advisory attacker identity and adds
  `linkage_mismatch` reason context

Mismatch is explanatory correlation only. It does not create a control path.

## Empty-State Behavior

When no selected fixture track exists, the surface renders:

`Select a track to inspect lineage and recommendation origin.`

No placeholder lineage, threat evaluation, or advisory origin data is shown. No
automatic selection is performed.

## Governance Boundaries

Track → Threat Traceability Layer V1 is read-only and recommendation-origin
explanation only.

Explicitly prohibited:

- telemetry changes
- bridge transport changes
- ROS topic changes
- schema changes
- parser-contract changes
- tactical controls
- assignment authority
- engagement authority
- autonomous behavior
- SA coupling
- automatic track selection
- command or approve/reject affordances

Governance copy remains visible in the workbench and integration header:

- `THREAT TRACEABILITY - read-only lineage explanation only; no assignment, engagement, or autonomy authority`
- Explanatory lineage only
- Recommendation origin visibility only
- No assignment authority
- No engagement authority

## Governance Audit (V1 Freeze)

Source inspection confirms:

- no telemetry channel additions
- no bridge transport modifications
- no ROS topic modifications
- no schema or parser-contract changes
- no tactical controls in traceability components
- no assignment, engagement, approve/reject, or autonomy controls
- no SA viewer or SA import coupling
- 32 passing traceability tests including read-only enforcement checks

## Limitations

V1 is intentionally fixture backed. It does not consume live:

- `entity_pose_mirror`
- `intelligence_advisory` transport
- `tactical_state`
- `tactical_recommendation`
- ROS `/tracks/state`
- SA replay bundles

The current fixture model demonstrates UI, linkage resolution, freshness
alignment, and governance boundaries. It does not prove live track-to-advisory
correlation, live stale alignment, or operational recommendation authority.

## Future Work

Future scoped phases may add additive read-only telemetry-backed assembly.
Any expansion must remain parser-safe and governance-bounded.

Possible future work:

- live track mirror + advisory transport assembly selectors
- explicit entity-to-track linkage contract (additive, read-only)
- stale alignment from transport `stale` and `stale_reason`
- provenance fixtures derived from controlled replay/runtime examples
- cross-panel navigation between Track & Sensor and Traceability workbenches

Future work must not imply telemetry, bridge, ROS, schema, tactical,
assignment, engagement, autonomy, or SA changes unless explicitly opened in a
separate scoped phase.

## Validation

Freeze validation:

- `npm test traceability`
- `npm run build`
- `git diff --check`
- `git status`
- governance audit by source inspection

The implementation adds UI-only explanatory rendering, local fixtures, selector
assembly, workstation placement, and tests. It does not introduce telemetry
channels, bridge transport changes, ROS topic changes, schema changes, tactical
controls, assignment actions, engagement actions, autonomy, or SA coupling.
