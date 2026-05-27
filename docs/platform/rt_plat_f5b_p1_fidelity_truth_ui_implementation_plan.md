# RT-F5b — Fidelity Truth UI (PLAT-RT-F5b P1)

**Phase:** PLAT-RT-F5b P1 — runtime fidelity cognition UI  
**Prerequisite:** PLAT-RT-F5b P0 frozen — [rt_plat_f5b_p0_freeze_audit.md](../evaluation/rt_plat_f5b_p0_freeze_audit.md)  
**Authority:** [rt_runtime_fidelity_cognition_v1.md](../evaluation/rt_runtime_fidelity_cognition_v1.md), [rt_runtime_fidelity_coupling_v1.md](../evaluation/rt_runtime_fidelity_coupling_v1.md)

## Goal

Surface truth-attested vs explanatory runtime information inside the RT workstation when `enable_fidelity_coupling=true`. Read-only cognition only — no registry rewrite, no SA viewer changes, no experiment metrics derive.

## Delivered (P1)

| Item | Location |
|------|----------|
| `build_fidelity_telemetry_fields()` | `platform/rt-sandbox-bridge/rt_sandbox/fidelity_coupling.py` |
| Pull passthrough on `world_summary` / `session_health` | `platform/rt-sandbox-bridge/rt_sandbox/telemetry_bridge.py` |
| `fidelityCognition.ts` | `platform/rt-sandbox-ui/src/fidelity/` |
| `FidelityTruthCognitionStrip` | `platform/rt-sandbox-ui/src/components/FidelityTruthCognitionStrip.tsx` |
| `BANNER_FIDELITY_TRUTH` | `platform/rt-sandbox-ui/src/governance/banners.ts` |
| Hub / Cesium / terrain / diagnostics wiring | `RuntimeCognitionHub.tsx`, `CesiumRuntimePanel.tsx`, `TerrainCognitionStrip.tsx`, `BackgroundDiagnostics.tsx` |
| Bridge + UI tests | `test_rt_sandbox_bridge.py`, `fidelityCognition.test.ts`, strip tests |

## Forbidden (unchanged)

- Bridge HTTP route / subcommand / adapter IPC changes
- `platform/sa-r0-viewer/` changes
- Browser `capture_session`; SA auto-import; M3 distributed queue
- `fidelityMetricsDerive.ts`, `rt_experiment_fidelity_metrics.py` (P2)
- Parser/topic/schema changes
- Tactical redesign; pose auto-correction from truth

## Validation

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
pytest src/counter_uas/test/test_rt_sandbox_bridge.py -k fidelity -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
```

## Stop line

PLAT-RT-F5b P1 frozen. Do not start **P2** (experiment fidelity metrics derive + compare strip) without separate wave audit.

## Related

- [rt_plat_f5b_p1_governance_review_r1.md](../evaluation/rt_plat_f5b_p1_governance_review_r1.md)
- [rt_plat_f5b_p1_freeze_audit.md](../evaluation/rt_plat_f5b_p1_freeze_audit.md)
- [rt_roadmap_plat_rt_f5b_v1.md](../evaluation/rt_roadmap_plat_rt_f5b_v1.md)
