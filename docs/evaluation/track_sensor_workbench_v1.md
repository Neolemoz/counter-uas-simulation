# Track & Sensor Workbench V1

Freeze ID: `PLAT-RT-TRACK-WB1`

Status: frozen

## Scope

Track & Sensor Workbench V1 is a read-only RT sandbox workstation surface for
inspecting fixture-backed track explanation records. It gives reviewers a
bounded place to inspect track detail, sensor contribution, lifecycle,
confidence, and advisory linkage without changing telemetry, bridge transport,
ROS topics, schemas, tactical controls, assignment behavior, engagement
behavior, autonomy, or SA/replay tooling.

This freeze covers:

- `TrackSensorWorkbench`
- `TrackDetailPanel`
- `SensorContributionPanel`
- `TrackLifecyclePanel`
- `TrackConfidencePanel`
- `TrackAdvisoryLinkPanel`
- `SelectedTrackSensorWorkbench`
- fixture-backed selected-track helpers
- workstation-level Track & Sensor Workbench placement
- empty selected-track state
- stale selected-track visibility
- component, fixture-binding, UX polish, and workstation placement tests

## Architecture

The workbench is implemented under:

- `platform/rt-sandbox-ui/src/tracks/workbench/`
- `platform/rt-sandbox-ui/src/workstation/AppWorkstationSlots.tsx`

The workstation-level placement is:

`AppWorkstationSlots -> TrackSensorWorkbenchWorkstationSurface -> SelectedTrackSensorWorkbench -> TrackSensorWorkbench`

The surface is separate from tactical and intelligence panels. It is rendered in
the workstation tactical column as its own read-only workbench surface, before
the tactical manual/assisted/autonomous controls. The workbench itself exposes
no buttons, callbacks, assignment controls, engagement controls, or autonomous
controls.

## Selected-Track Flow

V1 is selected-track driven and fixture backed:

1. `TrackSensorWorkbenchWorkstationSurface` receives the current selected ID as
   `selectedTrackId`.
2. `SelectedTrackSensorWorkbench` calls
   `getSelectedTrackSensorWorkbenchModel`.
3. The selector resolves that ID against local
   `TRACK_SENSOR_WORKBENCH_FIXTURES`.
4. Matching fixture records render the full workbench.
5. Missing or empty selections render the empty state.

There is no telemetry subscription, no bridge request, no ROS topic read, no
automatic track selection, and no generated placeholder data.

## Workstation Placement

The workstation surface is mounted inside the existing connected workstation
area in `AppWorkstationSlots.tsx`. It is intentionally independent from:

- `IntelligenceAdvisoryWorkstationSurfaces`
- `SelectedTargetAdvisoryCard`
- `ThreatEvaluationWorkbench`
- `TacticalManualPanel`
- `TacticalAssistedPanel`
- `TacticalAutonomousPanel`

The surface can render:

- selected active fixture track
- selected stale fixture track
- selected fixture track with no advisory link
- empty state when the selected ID has no fixture-backed track

## Track Detail Panel

`TrackDetailPanel` displays the selected fixture track state:

- `track_id`
- `linked_entity_id`
- `track_state`
- pose
- velocity
- heading
- speed
- `source_authority`
- `last_update`
- `track_age`
- staleness

The panel is display-only. Values come from the selected fixture model and are
not inferred from live telemetry.

## Sensor Contribution Panel

`SensorContributionPanel` renders four explanation rows:

- radar
- camera
- fused detection
- tracker update

Each row displays:

- source
- status
- freshness
- contribution
- agreement
- notes

Sensor contribution is explanatory input visibility only. It is not sensor
truth, sensor coverage proof, a detection contract, or a ROS topic/schema
change.

## Lifecycle Panel

`TrackLifecyclePanel` renders a read-only lifecycle timeline. Core events are:

- first seen
- confirmed
- updated
- coasted
- dropped

Optional events include:

- missed
- reacquired
- merged
- stale

Missing core events render as unavailable explanation rows instead of hiding the
timeline. Lifecycle data is fixture explanation only in V1.

## Confidence Panel

`TrackConfidencePanel` displays bounded track-quality confidence:

- confidence score
- confidence level
- factor chips
- basis text

The panel explicitly states that this is:

- not mission success confidence
- not kill probability
- not engagement confidence

Confidence is not assignment authority, engagement authority, readiness scoring,
or autonomy authorization.

## Advisory Link Panel

`TrackAdvisoryLinkPanel` displays the read-only explanation chain:

`Track -> Threat Evaluation -> Advisory`

Displayed fields:

- `track_id`
- `attacker_id`
- `threat_rank`
- `threat_score`
- `recommended_defender`
- `freshness_alignment`

If a selected fixture has no advisory link, the panel displays
`No advisory link available for this track.` No advisory is synthesized.

## Stale-Track Behavior

Stale selected tracks remain visible. When `track.staleness` is `stale`, the
workbench:

- displays a stale banner
- keeps the compact context header visible
- keeps track detail visible
- keeps sensor contribution visible
- keeps lifecycle visible
- keeps confidence visible
- keeps advisory linkage visible when available

Stale data remains historical/explanatory only. It does not grant command
authority or mutate track/advisory state.

## Empty-State Behavior

When no selected fixture track exists, the surface renders:

`Select a track to inspect sensor and track details.`

No placeholder track, sensor, lifecycle, confidence, or advisory data is shown.
No automatic selection is performed.

## Governance Boundaries

Track & Sensor Workbench V1 is read-only and recommendation-only.

Explicitly prohibited:

- telemetry changes
- bridge transport changes
- ROS topic changes
- schema changes
- parser-contract changes
- tracker redesign
- fusion redesign
- tactical controls
- assignment authority
- engagement authority
- autonomous behavior
- SA coupling
- automatic track selection
- sensor-truth claims without explicit future source data

The workbench governance copy remains visible in the sticky header:

`TRACK & SENSOR WORKBENCH - read-only explanation only; no assignment, engagement, or autonomy authority`

## Limitations

V1 is intentionally fixture backed. It does not consume:

- `entity_pose_mirror`
- `world_summary`
- `tactical_state`
- `tactical_recommendation`
- `intelligence_advisory`
- ROS `/radar/detections`
- ROS `/camera/detections`
- ROS `/fused_detections`
- ROS `/tracks`
- ROS `/tracks/state`

The current fixture model demonstrates the UI and governance boundary. It does
not prove live sensor provenance, live track quality, live covariance quality,
or live lifecycle continuity.

## Future Work

Future scoped phases may design an additive read-only track explanation data
contract. Any future expansion must remain parser-safe and governance-bounded.

Possible future work:

- explicit entity-to-track linkage model
- additive read-only track explanation mirror
- sensor contribution records with source freshness
- covariance quality summaries
- lifecycle counters and event history
- stale alignment between track and advisory state
- provenance fixtures derived from controlled replay/runtime examples

Future work must not imply telemetry, bridge, ROS, schema, tracker, fusion,
tactical, assignment, engagement, autonomy, or SA changes unless explicitly
opened in a separate scoped phase.

## Validation

Freeze validation:

- `npm test tracks`
- `npm run build`
- `git diff --check`
- `git status`
- governance audit by source inspection

The implementation adds UI-only explanatory rendering, local fixtures, selector
helpers, workstation placement, and tests. It does not introduce telemetry
channels, bridge transport changes, ROS topic changes, schema changes, tactical
controls, assignment actions, engagement actions, autonomy, or SA coupling.
