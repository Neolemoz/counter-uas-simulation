# Threat Evaluation Workbench V1

Freeze ID: `PLAT-RT-THREAT-WB1`

Status: frozen

## Scope

Threat Evaluation Workbench V1 is a read-only RT sandbox workstation surface for
explaining an already-derived `rt_intelligence_advisory_v1` advisory. It makes
existing advisory evidence easier to inspect without changing advisory
computation, telemetry transport, schemas, tactical commands, assignments,
engagement behavior, autonomy, ROS/Gazebo behavior, or SA/replay coupling.

This freeze covers:

- `ThreatEvaluationWorkbench`
- `ThreatBreakdownPanel`
- `DefenderComparisonPanel`
- `RecommendationExplanationPanel`
- `ConfidenceExplanationPanel`
- workstation-level selected-advisory placement
- selected-advisory empty state
- stale advisory explanation visibility
- component and workstation integration tests

## Architecture

The workbench is implemented under:

- `platform/rt-sandbox-ui/src/intelligence/workbench/`
- `platform/rt-sandbox-ui/src/intelligence/SelectedTargetAdvisoryCard.tsx`
- `platform/rt-sandbox-ui/src/intelligence/intelligenceSelectors.ts`
- `platform/rt-sandbox-ui/src/workstation/AppWorkstationSlots.tsx`

The workstation-level placement lives inside the existing intelligence
workstation surface:

`AppWorkstationSlots -> IntelligenceAdvisoryWorkstationSurfaces -> ThreatEvaluationWorkbenchSurface -> ThreatEvaluationWorkbench`

`SelectedTargetAdvisoryCard` remains a compact selected-advisory summary. The
workbench is a separate workstation-level panel, not nested in the selected
card.

## Advisory Flow

The workbench consumes the existing `intelligence_advisory` UI snapshot through
existing selector helpers. It is selected-advisory driven:

1. The existing `intelligence_advisory` telemetry snapshot is parsed as
   `rt_intelligence_advisory_transport_v1`.
2. `getSelectedEntityAdvisory` finds an advisory whose `identity.attacker_id`
   matches the selected entity ID.
3. The standalone workbench renders only that selected advisory.
4. No global advisory is automatically selected.
5. When no matching advisory exists, the workbench renders the empty state.

The workbench does not request, mutate, derive, or publish telemetry. It does
not alter the advisory transport payload or backend advisory engine.

## Workbench Sections

### ThreatBreakdownPanel

Displays the existing `threat_evaluation.threat_components` fields:

- factor
- raw value
- normalized value
- weight
- UI-derived contribution

### DefenderComparisonPanel

Displays existing `defender_ranking.ranked_defenders` rows:

- rank
- defender ID
- feasibility
- TTI
- reason-code chips

The panel preserves payload order and does not re-rank defenders in the browser.

### RecommendationExplanationPanel

Displays existing recommendation and reasoning fields:

- `recommended_defender.defender_id`
- feasibility state and reason
- recommended TTI
- `reasoning.explanation`
- `reasoning.reason_codes`

### ConfidenceExplanationPanel

Displays existing heuristic confidence fields:

- `confidence.heuristic_confidence.score`
- `confidence.heuristic_confidence.level`
- `confidence.heuristic_confidence.basis`

The panel explicitly states that confidence is heuristic advisory quality only,
not mission success confidence and not kill probability.

## Threat Breakdown Model

Threat contribution is derived in the UI only:

`contribution = normalized * weight`

This derived value is display-only. It is not a new backend field, schema
contract, telemetry field, probability, kill likelihood, readiness score, or
operational risk estimate.

The component uses only existing component values:

- `distance_to_protected_center`
- `best_feasible_tti`
- `descent_factor`
- `critical_zone_factor`

## Defender Comparison Model

The defender comparison table uses `ranked_defenders` as provided by
`rt_intelligence_advisory_v1`.

The UI does not:

- recompute feasibility
- recompute TTI
- create assignment candidates
- promote a defender to an assignment
- expose approval or reject controls
- issue tactical commands

Rank `#1` is displayed as advisory ranking only, not assignment authority.

## Recommendation Explanation

Recommendation explanation is display-only. It presents existing advisory
reasoning and recommendation details for reviewer cognition. It does not connect
to tactical assisted approval, manual assignment, engagement, or autonomous
behavior.

No-solution advisories remain visible and use the existing no-solution copy.

## Confidence Explanation

`heuristic_confidence` is advisory quality confidence only. It is explicitly not:

- mission success confidence
- kill probability
- engagement confidence
- authorization confidence
- readiness scoring

The workbench displays positive basis entries already present in the advisory
payload. It does not infer missing evidence as new schema data.

## Stale Advisory Behavior

The workbench supports stale advisory inspection at the workstation surface.
When the transport is stale, the workbench:

- remains visible for the selected stale advisory
- labels the advisory as stale
- preserves all explanation sections
- displays stale reason when available

Stale data remains historical/explanatory only. Staleness does not grant command
authority or mutate advisory contents.

## Empty-State Behavior

When no selected entity matches an advisory, the standalone workbench renders:

`Select an attacker to inspect threat evaluation details.`

No placeholder advisory data is shown. No automatic attacker selection is
performed.

## Governance Boundaries

Threat Evaluation Workbench V1 is read-only and recommendation-only.

Explicitly prohibited:

- telemetry changes
- advisory transport changes
- schema changes
- parser/topic changes
- ROS or Gazebo integration changes
- tactical command changes
- assignment authority
- engagement authority
- weapon authority
- autonomous behavior
- SA coupling
- automatic attacker selection
- approve/reject controls
- operational readiness scoring

The existing advisory banner remains visible:

`INTELLIGENCE ADVISORY - recommendation only; no assignment, engagement, or weapon authority`

## Validation

Freeze validation:

- `npm test intelligence`
- `npm run build`
- `git diff --check`
- governance audit by source inspection

The implementation adds UI-only explanatory rendering and tests. It does not
modify `rt_intelligence_advisory_transport.py` or introduce new telemetry
channels.

## Limitations

Current limitations are intentionally frozen:

- The workbench is selected-advisory driven only.
- It does not provide global queue navigation.
- It does not explain cross-attacker rank deltas beyond displaying the selected
  advisory fields.
- It does not expose unavailable/missing confidence basis as structured negative
  evidence.
- It does not add backend reason codes or defender infeasibility details.
- It does not add Cesium overlays or map geometry.

## Future Work

Future scoped waves may consider:

- advisory-list selection ergonomics without automatic selection
- richer cross-attacker comparison, if backed by additive advisory data
- explicit missing-basis display, if backed by a frozen schema extension
- documentation screenshots or demo review workflow updates
- density polish for large defender lists

Any future telemetry, schema, transport, tactical-control, assignment,
engagement, autonomy, ROS/Gazebo, or SA coupling change requires a separate
scoped governance wave.

## Frozen Verdict

`PLAT-RT-THREAT-WB1` is frozen as a workstation-level, selected-advisory,
read-only explanation workbench for existing RT intelligence advisory payloads.
It adds no command authority, no telemetry authority, no assignment authority,
no engagement authority, and no autonomy.
