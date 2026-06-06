# RT Protected Center UI V1

Freeze ID: `PLAT-RT-PROTECTED-CENTER1`

Status: frozen

## Purpose

Protected Center UI V1 adds **explicit, operator-initiated** protected-center
designation ergonomics and read-only status cognition to the RT sandbox
workstation. It consumes the existing bridge command
`designate_protected_center` (frozen in `PLAT-RT-INTEL-LIVE1`) and the
existing `protected_center_unavailable` advisory stale reason. It does **not**
introduce assignment, engagement, autonomy, tactical coupling, telemetry schema
changes, or backend contract changes.

## Designation workflow

Designation is **always explicit**:

1. Operator selects an entity in the world editor or Cesium map.
2. Operator clicks **Designate Protected Center** (WorldEditingGrid selected
   bar or Cesium selected-entity action row).
3. UI calls `designateProtectedCenter(sessionId, entityId, replace?)` via
   `POST /v1/command` (`commandType: designate_protected_center`).
4. On success, App-level session cache updates `protectedCenterEntityId` and
   triggers `doPull()`.

Replace flow:

- When a different center is already designated, `window.confirm` asks before
  sending `replace: true` (shared helper:
  `resolveProtectedCenterDesignationAttempt` / `executeProtectedCenterDesignation`).

Forbidden designation triggers:

- No auto-designation on spawn.
- No auto-designation on scenario apply.
- No designation from hover alone.
- No designation from selection alone (button click required).
- No inference from defense-zone geometry or `waypoint_marker` type alone.

## Protected-center status model

**App authority (UI cache):**

- `protectedCenterBySession: Record<sessionId, entityId | null>` in `App.tsx`
- Derived per active session as `protectedCenterEntityId`
- Cleared on: reset session, apply to runtime, delete of designated entity,
  `protected_center_unavailable` advisory stale, session disconnect prune

**Recovery notices:**

- `protectedCenterRecoveryNoticeBySession` records why designation was cleared
  (`reset_session`, `apply_scenario`, `protected_center_unavailable`)
- Cleared on successful re-designation
- Surfaces via `ProtectedCenterRecoveryBanner` when no center is designated

**UI-cache authority limitation:**

The workstation displays designation status from the **UI session cache** after
a successful command. The bridge OK response does not currently return
`protected_center_entity_id` in the HTTP payload; pull does not yet hydrate
the cache from session state. Backend session record remains authoritative for
live intelligence assembly (`PLAT-RT-INTEL-LIVE1`); UI cache may diverge until
re-designation or unavailable stale clears it. Future pull-sync is **docs-only**
— requires a separate scoped wave.

## Cross-surface consistency

All surfaces consume the same App props — **no local designation state**:

| Surface | Component | Props |
|---------|-----------|-------|
| Intelligence column | `ProtectedCenterStatusStrip` | `protectedCenterEntityId`, `protectedCenterRecoveryNotice` |
| World editor | `WorldEditingGrid` | same + `onDesignateProtectedCenter` |
| Cesium | `CesiumRuntimePanel` / `CesiumSelectedEntityActionRow` | same |
| Defense zones | `defenseZoneOptions`, `DefenseZoneSvgOverlay` | `protectedCenterEntityId` |
| Entity markers | `syncEntityMarkers` | `protectedCenterEntityId` |

## Visual emphasis (Cesium + SVG)

Shared resolver: `defenseZoneVisualState.ts` → `resolveDefenseZoneEntityVisual`.

- Designated center: full opacity, higher emphasis, emerald **Protected center**
  label, Cesium emerald halo (distinct from amber selection ring).
- Non-designated `waypoint_marker` candidates: remain visible at reduced fade.
- Selection ≠ designation (explicit ID match only via `isDesignatedProtectedCenter`).

## Unavailable recovery UX

When `intelligence_advisory` transport is stale with
`stale_reason: protected_center_unavailable`:

- `IntelligenceAdvisoryPanel` / `IntelligenceAdvisoryStrip` show recovery copy
  (not generic advisory stale wording).
- App clears UI designation cache and sets recovery notice
  `protected_center_unavailable`.
- `ProtectedCenterRecoveryBanner` guides re-designation.

## Forbidden scope

This freeze does **not** authorize:

- Auto-designation (spawn, scenario, selection, hover, tactical)
- Telemetry channel or schema changes
- Backend bridge / session handler changes
- Tactical assign / engage / autonomous loops
- Assignment or engagement authority
- SA viewer live hooks or import automation
- Pull-sync hydration implementation (documented future hook only)
- Parser / ROS / topic changes

## Implementation map

- `platform/rt-sandbox-ui/src/bridge/protectedCenterCommands.ts`
- `platform/rt-sandbox-ui/src/intelligence/protectedCenter*.ts(x)`
- `platform/rt-sandbox-ui/src/components/DesignateProtectedCenterButton.tsx`
- `platform/rt-sandbox-ui/src/components/CesiumSelectedEntityActionRow.tsx`
- `platform/rt-sandbox-ui/src/cesium/defenseZoneVisualState.ts`
- `platform/rt-sandbox-ui/src/App.tsx` (session cache + recovery notices)
- `platform/rt-sandbox-ui/src/workstation/AppWorkstationSlots.tsx`

## Validation

- `npm test intelligence` (includes protected-center integration tests)
- `npm run build`
- `git diff --check`
