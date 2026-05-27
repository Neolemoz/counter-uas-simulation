# RT-G4 — Runtime Telemetry Bridge Freeze Audit (PLAT-RT-G4)

## Scope

- [rt_g4_telemetry_bridge_plan.md](../platform/rt_g4_telemetry_bridge_plan.md)
- [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md)
- `platform/rt-sandbox-bridge/rt_sandbox/telemetry_bridge.py`
- Extended `adapter_worker.py`, `runtime_adapter.py`, `session_manager.py`, `telemetry_subscriptions.py`, `governance.py`
- `scripts/rt/rt_adapter_inspect.py` (`telemetry-status`)
- `src/counter_uas/test/test_rt_sandbox_bridge.py`
- [rt_g4_governance_review_r1.md](rt_g4_governance_review_r1.md)

Not in scope: SA viewer, G5 capture normalization, new telemetry channels, rosbridge.

Prerequisite: PLAT-RT-G3 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-G4.

## Boundary Checks

| Check | Result |
|-------|--------|
| SA viewer unchanged | Pass |
| Parser/topic safety | Pass |
| Default stub path | Pass |
| Mock adapter CI | Pass |
| EntityRegistry authority | Pass |
| PoseSyncMirror sync authority | Pass |
| Adapter-fed `entity_pose_mirror` when adapter on | Pass |
| Telemetry stale non-blocking | Pass |
| No federation writes | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `telemetry_bridge.py` | Yes |
| 2 | `poll_telemetry` IPC | Yes |
| 3 | Channel payload resolver | Yes |
| 4 | Telemetry audit events | Yes |
| 5 | Ring buffer trim audit | Yes |
| 6 | `adapter_poll_telemetry` sub-command | Yes |
| 7 | `telemetry-status` inspect | Yes |
| 8 | Governance review R1 | Yes |
| 9 | Freeze audit (this document) | Yes |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

75 tests passed at freeze time (67 prior + 8 G4).

## Stop Line

Do not implement PLAT-RT-G5 capture normalization or SA replay ingestion without a new scoped wave and freeze audit.
