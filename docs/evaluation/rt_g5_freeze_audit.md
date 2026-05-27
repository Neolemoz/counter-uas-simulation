# RT-G5 — Runtime Capture Normalization Freeze Audit (PLAT-RT-G5)

## Scope

- [rt_g5_capture_normalization_plan.md](../platform/rt_g5_capture_normalization_plan.md)
- [rt_capture_normalization_v1.md](rt_capture_normalization_v1.md)
- `platform/rt-sandbox-bridge/rt_sandbox/capture_normalize.py`
- Extended `capture.py`, `export_boundary.py`, `session_manager.py`, `telemetry_bridge.py`, `governance.py`
- `scripts/rt/rt_capture_inspect.py`, `scripts/rt/rt_capture_normalize.py`, `scripts/rt/rt_capture_approve.py`
- `src/counter_uas/test/test_rt_sandbox_bridge.py`
- [rt_g5_governance_review_r1.md](rt_g5_governance_review_r1.md)

Not in scope: SA viewer, automatic replay ingestion, federation writes, rosbridge, new bridge commands.

Prerequisite: PLAT-RT-S5, PLAT-RT-G3, PLAT-RT-G4 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-G5.

## Boundary Checks

| Check | Result |
|-------|--------|
| SA viewer unchanged | Pass |
| Parser/topic safety | Pass |
| Default stub path | Pass |
| Mock adapter CI | Pass |
| EntityRegistry authority | Pass |
| No federation writes | Pass |
| No auto SA import | Pass |
| Normalization at capture pre-teardown | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `capture_normalize.py` | Yes |
| 2 | `TelemetryMirror.summary()` | Yes |
| 3 | Capture-time normalization integration | Yes |
| 4 | Export audit events (4 types) | Yes |
| 5 | `rt_capture_normalize.py` | Yes |
| 6 | `rt_capture_inspect normalization-status` | Yes |
| 7 | Approval normalization gate | Yes |
| 8 | Contract `rt_capture_normalization_v1.md` | Yes |
| 9 | Governance review R1 | Yes |
| 10 | Freeze audit (this document) | Yes |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

84 tests passed at freeze time (75 prior + 9 G5).

## Stop Line

Do not implement automatic SA replay ingestion, federation publication, or SA viewer runtime integration without a new scoped wave and freeze audit.
