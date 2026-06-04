# Intelligence Advisory UI V1

Freeze ID: `PLAT-RT-INTEL-UI1`

Status: frozen

## Scope

Intelligence Advisory UI V1 displays `rt_intelligence_advisory_transport_v1`
telemetry in the RT sandbox UI. It is a read-only cognition surface for the
already-frozen `PLAT-RT-INTEL1` advisory layer.

This wave adds no runtime behavior, no tactical command actions, no Cesium
advisory overlays, no assignment controls, no autonomous behavior, and no SA
coupling.

## Architecture

The UI layer is split into:

- telemetry typing and snapshot flow
- TypeScript advisory contracts
- selector helpers
- read-only advisory components
- workstation placement

The implementation lives under:

- `platform/rt-sandbox-ui/src/intelligence/`
- `platform/rt-sandbox-ui/src/telemetry/constants.ts`
- `platform/rt-sandbox-ui/src/telemetry/channelIndex.ts`
- `platform/rt-sandbox-ui/src/components/CesiumRuntimePanel.tsx`
- `platform/rt-sandbox-ui/src/workstation/AppWorkstationSlots.tsx`

## Telemetry Flow

The UI subscribes to the additive backend telemetry channel:

`intelligence_advisory`

The channel payload is typed as `RtIntelligenceAdvisoryTransportV1` and is
stored in the existing workstation snapshot map:

`snapshots.intelligence_advisory`

No new backend channel, parser contract, ROS topic, or Gazebo integration is
introduced by this UI freeze.

## Selectors

Selectors are pure helpers:

- `getTopAdvisory`
- `getSelectedEntityAdvisory`
- `getRankedAdvisories`
- `getAdvisoryConfidenceLabel`
- `getAdvisoryReasonLabels`
- `getAdvisoryTransportFromSnapshot`

Selectors treat stale transport as inactive for ranked/top/selected advisory
lookups. Null ranks sort last. Confidence labels are heuristic advisory labels
only and do not represent mission success, probability of kill, or engagement
confidence.

## Advisory Panel

`IntelligenceAdvisoryPanel` appears in the RT tactical/workstation column when
`intelligence_advisory` transport exists.

It displays:

- advisory count
- empty/stale/active state
- ranked advisories
- threat rank
- threat score
- recommended defender
- TTI
- confidence label
- reason labels

It has no buttons, no callbacks, and no assignment controls.

## Advisory Strip

`IntelligenceAdvisoryStrip` appears near the existing Cesium tactical summary
area. It displays at-a-glance cognition:

- top advisory
- advisory count
- stale state
- governance copy

It does not create Cesium entities, overlays, labels, or map geometry.

## Selected-Target Advisory Card

`SelectedTargetAdvisoryCard` appears in the tactical/workstation column only
when the selected entity ID matches an advisory attacker ID.

It displays:

- selected attacker advisory
- recommended defender
- TTI
- confidence label
- reason labels
- explanation

It remains hidden when no matching attacker advisory exists.

## Governance Boundaries

The Intelligence Advisory UI is recommendation only.

Explicitly prohibited:

- assignment mutation
- approve/reject workflow changes
- tactical command actions
- engagement authority
- weapon authority
- autonomous behavior
- Cesium advisory overlays
- SA coupling
- parser/topic changes

The frozen UI copy states:

`INTELLIGENCE ADVISORY - recommendation only; no assignment, engagement, or weapon authority`

## Limitations

The UI displays only telemetry already present in
`rt_intelligence_advisory_transport_v1`.

It does not derive advisories in the browser. It does not infer assignments from
advisory recommendations. It does not connect advisory recommendations to
existing tactical approve/reject or assignment flows.

## Future Work

Future scoped waves may consider:

- visual polish for dense multi-attacker advisory lists
- optional filtering by selected attacker
- advisory history or freshness timeline
- additional read-only map cognition

Any future assignment, approval, tactical command, autonomous behavior, parser,
SA, or Cesium overlay expansion requires a separate governance-scoped freeze.
