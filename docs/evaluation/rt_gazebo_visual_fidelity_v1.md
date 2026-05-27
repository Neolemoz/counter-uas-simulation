# RT Gazebo Visual Fidelity Policy (`rt_gazebo_visual_fidelity_v1`)

**Phase:** PLAT-RT-G6  
**Authority:** [rt_g6_gazebo_runtime_fidelity_plan.md](../platform/rt_g6_gazebo_runtime_fidelity_plan.md)

Visual placement and model policy for RT sandbox live Gazebo mode. Explanatory only — not SA replay authority.

---

## 1. World

| Property | Value |
|----------|-------|
| World file | `rt_sandbox_flat.sdf` |
| Extent | Flat ground ±1000 m (matches RT `WORLD_BOUNDS`) |
| Physics | Static entity proxies; no autonomous motion |
| Overlays | None — no danger domes, engagement markers, or tactical UI |

---

## 2. Entity visual proxies

| `entity_type` | Model | Color cue |
|---------------|-------|-----------|
| `drone` | Box 1×1×1 m | Green |
| `radar` | Cylinder r=0.8 h=2 m | Blue |
| `interceptor` | Box 1.2×0.6×0.6 m | Red |
| `waypoint_marker` | Pin/sphere r=0.4 m | Yellow |

Models are **non-tactical** placeholders for placement review only.

---

## 3. Ground snap

When `entity_ground_snap_enabled=true` (default):

- Apply model-specific z offset so footprint sits on ground plane (z=0)
- Command pose z is interpreted as entity center; snap adjusts for model half-height
- Registry command pose is unchanged; snap applies at Gazebo apply boundary only

---

## 4. Sync responsiveness

| Mechanism | Target |
|-----------|--------|
| Event-driven pose apply | On spawn/move/delete |
| `entity_state` publish | 10 Hz cap while entities exist |
| Optional background poll | `adapter_live_background_poll_hz` (default 0) |

Drift beyond `pose_sync_drift_threshold_m` → `SYNC_STALE` (blocking on entity ops, unchanged G3 policy).

---

## 5. Maintainer review

- `use_gazebo_gui:=true` on launch for local visual inspection
- Headless default for CI and unattended runs
- Cesium globe remains explanatory mirror — Gazebo window is sim truth in live mode

---

## Related

- [rt_adapter_live_sync_v1.md](rt_adapter_live_sync_v1.md)
- [rt_cesium_interactive_editing_ui_v1.md](rt_cesium_interactive_editing_ui_v1.md)
