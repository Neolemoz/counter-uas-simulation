# RT-F5b P1 — Freeze Audit (PLAT-RT-F5b P1)

**Phase:** PLAT-RT-F5b P1 — fidelity truth UI  
**Status:** frozen

**Plan:** [rt_plat_f5b_p1_fidelity_truth_ui_implementation_plan.md](../platform/rt_plat_f5b_p1_fidelity_truth_ui_implementation_plan.md)

**P0 audit:** [rt_plat_f5b_p0_freeze_audit.md](rt_plat_f5b_p0_freeze_audit.md)

---

## Scope delivered

| # | Item | Done |
|---|------|------|
| 1 | `build_fidelity_telemetry_fields()` | `fidelity_coupling.py` |
| 2 | Pull passthrough on `world_summary` / `session_health` | `telemetry_bridge.py` |
| 3 | `fidelityCognition.ts` helpers | `platform/rt-sandbox-ui/src/fidelity/` |
| 4 | `FidelityTruthCognitionStrip` | `components/FidelityTruthCognitionStrip.tsx` |
| 5 | `BANNER_FIDELITY_TRUTH` | `governance/banners.ts` |
| 6 | `RuntimeCognitionHub` fidelity line + strip | Yes |
| 7 | `CesiumRuntimePanel` banner + strip + dual AGL terrain | Yes |
| 8 | `BackgroundDiagnostics` per-session fidelity label | Yes |
| 9 | Bridge + UI tests | Yes |
| 10 | Governance + registry | Yes |

**Not delivered (P2):** `fidelityMetricsDerive.ts`, `rt_experiment_fidelity_metrics.py`, experiment fidelity compare strip.

---

## Truth UI surfaces summary

| Surface | What it shows |
|---------|----------------|
| `RuntimeCognitionHub` | `Fidelity: truth_attested (sim)` / off line; compact truth strip with freshness, LOS/dome, drift summary |
| `FidelityTruthCognitionStrip` | Truth vs explanatory badges; stale / drift / divergence / partial_truth indicators |
| `TerrainCognitionStrip` | Dual AGL — `display_agl_m (explanatory)` + `sim_agl_m (truth_attested)` when coupling on |
| `CesiumRuntimePanel` | Fidelity banner; command vs truth pose readout for selected entity |
| `BackgroundDiagnostics` | Per-session `fidelity: on\|off\|stale\|unavailable` |

---

## Boundary guarantees

- No bridge HTTP route / subcommand / adapter IPC changes
- Additive pull metadata only on existing channels
- No parser/topic/schema changes
- No SA viewer or auto-import
- PLAT-RT-F5b P1 ≠ registry RT-1..7 realism waves
- Truth cognition never rewrites registry or auto-corrects display pose

---

## Regression evidence

```bash
python3 scripts/rt/lint_rt_runtime_subcommands.py --check
pytest src/counter_uas/test/test_rt_sandbox_bridge.py -k fidelity -q
cd platform/rt-sandbox-ui && npm ci && npm test && npm run build
```

---

## Recommended PLAT-RT-F5b P2 scope (advisory)

See [rt_roadmap_plat_rt_f5b_v1.md](rt_roadmap_plat_rt_f5b_v1.md) § P2:

- `fidelityMetricsDerive.ts` + golden parity
- `rt_experiment_fidelity_metrics.py` maintainer CLI
- Experiment fidelity compare strip (read-only badges)

**Not authorized** by this freeze.

---

## Stop line

PLAT-RT-F5b P1 frozen. Do not start P2 without governance review + freeze audit.
