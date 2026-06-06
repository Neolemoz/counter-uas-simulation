# RT Protected Center UI V1 Freeze Audit

Freeze ID: `PLAT-RT-PROTECTED-CENTER1`

Status: frozen

## Architecture summary

Protected Center UI V1 is a **UI-only workstation wave** layered on frozen
bridge designation (`designate_protected_center`) and frozen live intelligence
assembly (`PLAT-RT-INTEL-LIVE1`). App.tsx holds session-scoped designation
cache and recovery notices; all workstation surfaces read the same props. Visual
emphasis uses a shared defense-zone visual resolver for Cesium layers, entity
marker halos, and SVG overlay parity.

```
Operator click (Grid or Cesium)
        ↓
protectedCenterDesignation.ts (confirm + command)
        ↓
protectedCenterCommands.ts → bridge designate_protected_center
        ↓
App protectedCenterBySession cache + doPull()
        ↓
Status strip / Grid / Cesium / defense zones / markers
```

## Implemented scope

| Step | Deliverable |
|------|-------------|
| 1–2 | Audit: backend designation exists; UI gap identified |
| 3 | `designateProtectedCenter` wrapper, status strip, unavailable copy, grid designate action |
| 4 | Cesium/SVG emphasis, shared `defenseZoneVisualState`, designated halo |
| 5 | Cesium selected-entity designate action, shared button + designation helper |
| 6 | Recovery banners (reset / apply / unavailable), cross-surface continuity tests |
| 7 | Contract + freeze audit + registry (this document) |

### Files (primary)

- `platform/rt-sandbox-ui/src/bridge/protectedCenterCommands.ts`
- `platform/rt-sandbox-ui/src/intelligence/protectedCenterCopy.ts`
- `platform/rt-sandbox-ui/src/intelligence/protectedCenterDesignation.ts`
- `platform/rt-sandbox-ui/src/intelligence/ProtectedCenterStatusStrip.tsx`
- `platform/rt-sandbox-ui/src/intelligence/ProtectedCenterRecoveryBanner.tsx`
- `platform/rt-sandbox-ui/src/components/DesignateProtectedCenterButton.tsx`
- `platform/rt-sandbox-ui/src/components/CesiumSelectedEntityActionRow.tsx`
- `platform/rt-sandbox-ui/src/cesium/defenseZoneVisualState.ts`
- `platform/rt-sandbox-ui/src/App.tsx`
- `platform/rt-sandbox-ui/src/workstation/AppWorkstationSlots.tsx`
- `platform/rt-sandbox-ui/src/components/WorldEditingGrid.tsx`
- `platform/rt-sandbox-ui/src/components/CesiumRuntimePanel.tsx`
- `platform/rt-sandbox-ui/src/components/DefenseZoneSvgOverlay.tsx`
- `platform/rt-sandbox-ui/src/cesium/defenseZoneLayer.ts`
- `platform/rt-sandbox-ui/src/cesium/entityMarkers.ts`
- Tests under `platform/rt-sandbox-ui/src/intelligence/` and related component tests

## Excluded scope

- Backend / bridge changes
- Telemetry schema or channel additions
- Auto-designation at spawn, scenario apply, or selection
- Tactical assign / engage / autonomy integration
- Pull-sync cache hydration from session pull (documented future hook only)
- SA import or replay viewer changes
- Parser / ROS / topic changes

## Governance verification

| Check | Result |
|-------|--------|
| Explicit designation only | Pass — button click + confirm replace |
| No auto-designation | Pass — no spawn/scenario/hover/selection paths set cache |
| No telemetry schema changes | Pass — UI-only; consumes existing channels |
| No backend contract changes | Pass — uses existing `designate_protected_center` |
| No tactical coupling | Pass — no tactical command wiring |
| No assignment authority | Pass |
| No engagement authority | Pass |
| No autonomy | Pass |

## Validation results

Freeze commit: `b4bc9c0`

Executed at freeze:

| Command | Result |
|---------|--------|
| `npm test intelligence` | 54 passed |
| `npm run build` | Success |
| `git diff --check` | Clean |

Integration coverage: `protectedCenterContinuityIntegration.test.tsx`
(designate → surfaces → reset → recovery banner).

## Future pull-sync note (documentation only)

A future scoped wave may hydrate `protectedCenterBySession` from session pull
or bridge response when `protected_center_entity_id` is exposed read-only in
HTTP pull payloads. That would reduce UI-cache divergence after external
session mutations. **Not authorized by this freeze** — requires separate PLAN
checklist and registry entry.

## Frozen verdict

`PLAT-RT-PROTECTED-CENTER1` is frozen as an explicit-designation, UI-cache
status and emphasis layer with recovery continuity UX. Re-open only via scoped
plan + registry update.
