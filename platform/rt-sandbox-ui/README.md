# RT Sandbox UI (PLAT-RT-T1 / T2 / T3 / T4 / T5 / SA2 / V1)

RT-only local dev **runtime workstation**. Pull telemetry, interactive world editing (SVG + Cesium), session workflow visibility, and capture/handoff pipeline guidance — **not** SA replay authority.

## Prerequisites

- Node 20+
- Python bridge running on loopback

## Dev workflow

```bash
# Terminal 1 — bridge
python3 scripts/rt/run_rt_bridge.py

# Terminal 2 — UI (port 5174)
cd platform/rt-sandbox-ui
npm install
npm run dev
```

Open http://127.0.0.1:5174 → **Start session & subscribe**.

## Workstation layout (PLAT-RT-T4)

1. **Session rail** — connect, pull refresh, workflow badges (lifecycle, paused, editing)
2. **World column** — entity palette + SVG grid (authoritative editing)
3. **Viz column** — Cesium read-only mirror
4. **Mirrors** — runtime cognition hub + telemetry channel panels
5. **Pipeline footer** — capture/handoff mirror + maintainer pipeline (read-only via `list_capture_handoff_status`)

## Telemetry (PLAT-RT-T1)

Pull refresh surfaces lifecycle, health, world summary, entity pose mirror, and clock mirror. Explanatory only — not replay authority.

## World editing (PLAT-RT-T2)

1. Select entity type in palette (radar, interceptor, drone, waypoint_marker)
2. Click empty grid cell to **spawn**
3. Drag marker to **move**
4. Select entity + **Delete** key or button to **remove**
5. Telemetry mirror updates via pull refresh (explanatory only)

## Cesium runtime view (PLAT-RT-T3 / T5 / V1)

**PLAT-RT-V1** adds larger markers with entity-id labels, vertical bounds cues, camera presets (reset / tight bounds / fit entities), session accent strip on the globe panel, and per-tab accent colors for multi-session chrome.

When connected, the Cesium panel shows a 3D globe with entity markers, bounds overlay, and camera helpers.

**PLAT-RT-T5 interactive editing:**

- Click empty globe to **spawn** (uses palette entity type)
- Click marker to **select** (shared with SVG grid)
- Drag marker to **move**
- **Delete selected** button or Delete key
- Reset camera, focus selected, optional follow selected

SVG grid and Cesium globe both issue the same registry commands for the active session.

## Capture / handoff (PLAT-RT-T4 + SA2)

The browser shows **session capture readiness** from lifecycle telemetry and a **read-only staging mirror** (`list_capture_handoff_status`) scoped per `session_id` — normalization, approval, review decision, and `workflow_phase` per capture. Multi-session tabs and background diagnostics show capture counts and handoff phase badges.

Maintainer writes remain CLI-only (`rt_capture_normalize.py`, `rt_handoff_review.py`, `rt_sa_import.py`). The UI does not call `capture_session` or perform SA corpus commits.

## Governance banners

- RT SANDBOX — experimental simulation; not operational state
- TRANSIENT RUNTIME ONLY — not replay authority
- NOT SA REPLAY AUTHORITY
- WORLD EDITING ACTIVE (when session connected)
- CESIUM RUNTIME VIEW (when session connected)
- INTERACTIVE EDITING (when session connected)

## Tests

```bash
npm test
npm run build
scripts/ci_eval.sh tier0-rt-ui
```

## Maintainer audit

```bash
python3 scripts/rt/rt_adapter_inspect.py telemetry-status
python3 scripts/rt/rt_adapter_inspect.py sync-status
```

## Isolation

- Separate from `platform/sa-r0-viewer/`
- Registry commands authoritative; telemetry mirror explanatory
- Vite proxy `/v1` → `127.0.0.1:18765`
