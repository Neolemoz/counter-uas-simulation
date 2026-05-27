# RT-T1 — Runtime Telemetry UI Freeze Audit (PLAT-RT-T1)

**Scope:** First RT-only browser telemetry visualization surface (expansion wave after P2).

## In scope

- Plan: [rt_t1_telemetry_ui_plan.md](../platform/rt_t1_telemetry_ui_plan.md)
- Contract: [rt_telemetry_ui_v1.md](rt_telemetry_ui_v1.md)
- UI package: `platform/rt-sandbox-ui/`
- Telemetry cognition helpers (TS)
- Governance chrome (three banners)
- CI tier: `tier0-rt-ui`
- Governance review: [rt_t1_governance_review_r1.md](rt_t1_governance_review_r1.md)

## Not in scope

- `platform/sa-r0-viewer/` changes
- Cesium / geospatial map
- Drag/drop entity editing (RT-T2)
- SA replay bridge implementation
- Bridge command or channel changes
- WebSocket push telemetry

**Prerequisite:** PLAT-RT-R3d frozen (P2 complete).

## Governance Result

**Verdict: frozen** for PLAT-RT-T1.

## Boundary Checks

| Boundary | Result |
|----------|--------|
| SA viewer untouched | Pass |
| Parser/topic unchanged | Pass |
| Stub path default preserved | Pass |
| Mirrors ≠ authority in UI copy | Pass |
| Federation/corpus writes | Pass — none |
| Loopback-only transport | Pass |
| Pull-only telemetry data plane | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `platform/rt-sandbox-ui/` | Yes |
| 2 | `rt_telemetry_ui_v1.md` | Yes |
| 3 | Governance banners (3) | Yes |
| 4 | Telemetry cognition helpers | Yes |
| 5 | Five channel panels + 2D grid | Yes |
| 6 | `tier0-rt-ui` CI tier | Yes |
| 7 | Isolation tests (Vitest + pytest) | Yes |

## Regression Evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
(cd platform/rt-sandbox-ui && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0
scripts/ci_eval.sh tier0-rt-ui
# tier0: OK
# tier0-rt-ui: OK
# 123 passed (bridge tests)
# 11 passed (Vitest)
```

## Telemetry UI architecture summary

| Layer | Role |
|-------|------|
| Vite dev proxy | Same-origin `/v1` → `127.0.0.1:18765` |
| Bridge client | `POST /v1/command`, `GET /v1/telemetry/pull` |
| Channel index | Latest event per channel (S4 pattern) |
| Cognition | `source`, `authority_label`, health badges |
| Panels | lifecycle, health, world, entity mirror, clock |

## Isolation guarantees

- Third sibling under `platform/` (alongside bridge and SA viewer)
- No imports from `platform/sa-r0-viewer/`
- `isolation.py` SA path blocking unchanged
- Persistent governance banners on all routes
- Adapter mirrors displayed as `explanatory_telemetry`

## Stop Line

PLAT-RT-T1 frozen. Next expansion requires explicit new wave audit:

| Frontier | Notes |
|----------|-------|
| **RT-T2** | Drag/drop + runtime world editing UI |
| **PLAT RT→SA bridge** | Manual import handoff implementation (R2f planning closed) |
| **Cesium runtime viz** | Separate expansion wave |

Do **not** start RT-T2, Cesium, or SA bridge implementation without new audit.
