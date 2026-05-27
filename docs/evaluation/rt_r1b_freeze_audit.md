# RT-R1b — Adapter Poll Consolidation Freeze Audit (PLAT-RT-R1b)

## Scope

- [rt_r1b_adapter_poll_consolidation_plan.md](../platform/rt_r1b_adapter_poll_consolidation_plan.md)
- [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md)
- `platform/rt-sandbox-bridge/rt_sandbox/adapter_poll.py`
- `platform/rt-sandbox-bridge/rt_sandbox/time_utils.py`
- Extended `adapter_sync.py`, `pose_sync.py`, `telemetry_bridge.py`, `session_manager.py`
- [rt_r1b_governance_review_r1.md](rt_r1b_governance_review_r1.md)

Not in scope: telemetry UI, Cesium, SA integration, template resync (R2d), session_manager decomposition (R3a).

Prerequisite: PLAT-RT-R1a frozen; PLAT-RT-G5 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-R1b.

## Boundary Checks

| Check | Result |
|-------|--------|
| SA viewer unchanged | Pass |
| Parser/topic safety | Pass |
| Command failure semantics preserved | Pass |
| P1 roadmap closure (R2a/R2b/partial R2c) | Pass |
| No new bridge commands | Pass |
| R1a authority labels preserved | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `adapter_poll.py` — unified poll tick | Yes |
| 2 | `time_utils.py` — shared stale helpers | Yes |
| 3 | `sync_audit_event()` helper | Yes |
| 4 | Session manager poll routing simplification | Yes |
| 5 | `rt_poll_sync_semantics_v1.md` | Yes |
| 6 | Cross-links in feedback/telemetry contracts | Yes |
| 7 | Governance review R1 | Yes |
| 8 | Freeze audit (this document) | Yes |
| 9 | Additive R1b tests (3) | Yes |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

91 passed at freeze time (88 prior + 3 R1b).

## Consolidation summary

- Single adapter poll tick owns feedback + telemetry IPC polls.
- Eliminated double telemetry poll on spawn/move entity mutations.
- Shared UTC stale helpers (`is_poll_stale`) for G3/G4 mirrors.
- Telemetry three-store model documented; `TelemetryBuffer` boundary clarified.

## Remaining P1/P2 risks

- Template adapter resync policy (R2d / R1-AUTH-04) — **closed by PLAT-RT-R2d**
- Capture pose cognition doc (R2e / R1-CAP-02)
- SA bridge planning (R2f / R1-SA-05)
- Session manager decomposition (R3a / R1-DEBT-01)
- See [rt_roadmap_post_g5_v1.md](rt_roadmap_post_g5_v1.md) P1/P2 tables

## Stop Line

Do not start R2d or expansion waves until PLAT-RT-R1b is frozen. PLAT-RT-G5 and post-R1 stop lines unchanged for SA ingestion, federation, and viewer runtime hooks.
