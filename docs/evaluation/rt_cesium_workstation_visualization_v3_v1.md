# RT Cesium / Workstation Visualization V3 Contract (`rt_cesium_workstation_visualization_v3_v1`)

**Phase:** PLAN-RT-V3 — Cesium and workstation visualization planning  
**Authority:** [rt_v3_runtime_visualization_fidelity_plan.md](../platform/rt_v3_runtime_visualization_fidelity_plan.md)  
**Companion:** [rt_runtime_visualization_fidelity_v3_v1.md](rt_runtime_visualization_fidelity_v3_v1.md)  
**Supplements:** [rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md), [rt_cesium_runtime_ui_v1.md](rt_cesium_runtime_ui_v1.md), [rt_multi_session_poll_policy_v1.md](rt_multi_session_poll_policy_v1.md)

Normative layout and cognition grouping for RT workstation + Cesium after V1/V2/F4/M3. **Planning only** — no implementation in PLAN wave.

---

## 0. Core invariant

| Rule | Detail |
|------|--------|
| Scope | `platform/rt-sandbox-ui/` workstation + Cesium panels |
| One globe | Selected session only — no background session entities ([rt_v1_runtime_visualization_v1.md](rt_v1_runtime_visualization_v1.md)) |
| Pull transport | Unchanged loopback HTTP — no browser→ROS |
| Editing lock | M2/M3 semantics preserved on globe chrome |

---

## 1. Panel layout extensions

### 1.1 Workstation zones (`RuntimeWorkstationShell`)

```text
+------------------------------------------------------------------+
| SessionTabBar | SessionWorkflowStrip                              |
+------------------------------------------------------------------+
| [Workflow / experiment strip - existing]                          |
+----------+-------------------------------------------+------------+
| Cognition|  CesiumRuntimePanel (globe + layer rail)  | Optional   |
| rail     |                                           | SVG edit   |
| (left)   |                                           | (T2/T5)    |
+----------+-------------------------------------------+------------+
| BackgroundDiagnostics (V3: compact row when policy on)            |
+------------------------------------------------------------------+
```

| Zone | V3 addition |
|------|-------------|
| Cognition rail | Fixed-width column for grouped strips (§2) |
| Layer rail | Inside `CesiumRuntimePanel` header: toggles from `rt_visual_layer_registry_v3` |
| Diagnostic strip | Optional 1-row summary above accordion (§4) |

### 1.2 `CesiumRuntimePanel` internal layout

```text
+------------------------------------------+
| Banners (stacked, scroll)                |
+------------------------------------------+
| Layer toggles | Session chrome (accent)  |
+------------------------------------------+
|                                          |
|              Globe viewer                |
|                                          |
+------------------------------------------+
| Camera presets | Editing cognition      |
+------------------------------------------+
```

| Element | Rule |
|---------|------|
| Layer toggles | Grouped by `terrain_context` / `visibility_context` / `sensor_context` |
| Session chrome | `Viewing session {shortId}` + accent swatch + lock icon when editing session |
| Banners | Additive `BANNER_VISIBILITY_V3` below frozen stack when V3 overlays on |

---

## 2. Visual cognition grouping (`RuntimeCognitionHub`)

### 2.1 Block order (top → bottom)

| Block id | Title | Collapse default |
|----------|-------|------------------|
| `session_line` | Active session + pull health | expanded |
| `terrain_block` | Terrain (explanatory) | expanded when any terrain layer on |
| `visibility_block` | Visibility (heuristic) | collapsed |
| `fidelity_block` | Fidelity (F5b labels) | collapsed unless coupling on |
| `sensor_block` | Sensor context (nominal) | collapsed |
| `authority_block` | Authority + stale (T1) | expanded |

### 2.2 Hub line examples

| Block | Example line |
|-------|----------------|
| terrain_block | `Ridge: NorthSpine · band 20m · contour off` |
| visibility_block | `Wedge: heuristic ±30° · LOS terrain_blocked (explanatory)` |
| fidelity_block | `Fidelity: truth_attested (sim)` or `Fidelity: off (stub)` |
| sensor_block | `Domes: 2 nominal · occlusion markers: 3` |

### 2.3 Strip placement

| Strip | Location |
|-------|----------|
| `TerrainCognitionStrip` | Inside `terrain_block` |
| `FidelityTruthCognitionStrip` | Inside `fidelity_block` |
| V3 grouped visibility summary | Inside `visibility_block` — text-only when overlays off |

---

## 3. Multi-session readability

Extends V1 `sessionVisualIdentity` and M3 tab UX.

| Mechanism | V3 rule |
|-----------|---------|
| Active session | Full accent on markers, bounds, session strip |
| Inactive slots | Globe shows **only** active session; tabs show accent for all slots |
| Inactive marker preview | **Forbidden** on globe |
| Tab ↔ globe | On tab switch, `flyOnSessionSwitch` (V1) + refresh layer toggle state per session (PLAT: session-scoped toggle memory optional, local only) |
| Editing lock | Lock icon on Cesium header when `editing_session_id` matches active |
| Background poll | M3 `shouldPullSlotInAutoRefresh` unchanged — V3 does not add cross-session pull |

### 3.1 Slot accent table (unchanged from V1)

| Slot index | Accent token |
|------------|----------------|
| 0 | amber |
| 1 | sky |
| 2 | violet |

---

## 4. Background diagnostic visibility

### 4.1 Compact diagnostic row (V3-new, default on in PLAT P2)

When ≥1 background session exists, show one row below globe:

| Chip | Source |
|------|--------|
| `last_pull` | M3 `pullAge` per slot |
| `stale` | Telemetry stale flag |
| `paused` | M3 background poll pause |

Clicking row expands full `BackgroundDiagnostics` accordion — does not change poll policy.

### 4.2 Visibility rules

| Condition | Show compact row |
|-----------|------------------|
| Only active session | Hide row |
| ≥1 background session | Show row |
| All slots stale | Row uses warn styling; banner stack unchanged |

### 4.3 Forbidden

- Diagnostic row must not trigger capture, import, or bridge commands
- No “session unhealthy — auto-reset” copy

---

## 5. Banner stack

| Banner id | When shown |
|-----------|------------|
| Frozen T1/T3/T4/T5/SA2/F4/F5b | Per existing contracts |
| `BANNER_VISIBILITY_V3` | Any of `visibility_wedge_v3`, `horizon_hint_v3`, `stacked_los_v3` on |
| `BANNER_TERRAIN` | V2 — unchanged |
| `BANNER_REALISM_F4` | F4 — unchanged |

**Text (proposed):** `Visibility overlays are heuristic sandbox cues — not sensor coverage or operational picture.`

---

## 6. PLAT touch map (advisory)

| File | V3 change class |
|------|-----------------|
| `App.tsx` | Wire grouped blocks; compact diagnostic row |
| `CesiumRuntimePanel.tsx` | Layer rail zones |
| `RuntimeWorkstationShell.tsx` | Cognition rail column |
| `RuntimeCognitionHub.tsx` | Block collapse + ordering |
| `BackgroundDiagnostics.tsx` | Chip helpers for compact row |
| `useRtSessionWorkspace.ts` | Read-only inputs for diagnostic chips |

---

## 7. Explicit non-goals

- SA viewer layout changes
- Experiment workbench panel redesign (X2 scope)
- Advisory / handoff panel changes (F6/F7)
- New bridge commands or camera persistence
- Multi-globe or picture-in-picture sessions

---

## Related

- [rt_runtime_visualization_fidelity_v3_v1.md](rt_runtime_visualization_fidelity_v3_v1.md)
- [rt_v3_architecture_review_r1.md](rt_v3_architecture_review_r1.md)
