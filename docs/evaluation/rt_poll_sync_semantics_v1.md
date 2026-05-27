# RT Poll & Sync Semantics (`rt_poll_sync_semantics_v1`)

**Phase:** PLAT-RT-R1b — adapter poll consolidation  
**Authority:** [rt_r1b_adapter_poll_consolidation_plan.md](../platform/rt_r1b_adapter_poll_consolidation_plan.md)

Clarifies polling vs sync, telemetry vs feedback, and stale vs mismatch terminology after PLAT-RT-R1b.

---

## 1. Polling vs sync

| Term | Meaning |
|------|---------|
| **Poll** | Synchronous IPC round-trip to adapter worker (`poll_feedback`, `poll_telemetry`). Event-driven on bridge commands — no background timer loop. |
| **Sync** | Bridge registry command → adapter apply → feedback mirror update. Post-entity sync runs feedback poll after spawn/move. |
| **Adapter poll tick** | `run_adapter_poll_tick()` — single ownership point for optional feedback + telemetry polls (`adapter_poll.py`). |

Session manager orchestrates **when** to tick; `adapter_poll.py` owns **how**.

---

## 2. Feedback vs telemetry

| Path | Mirror | IPC op | Primary audit events |
|------|--------|--------|----------------------|
| **Feedback (G3)** | `PoseSyncMirror` | `poll_feedback` | `sync_update`, `sync_stale`, `sync_mismatch`, `adapter_feedback_lost` |
| **Telemetry (G4)** | `TelemetryMirror` | `poll_telemetry` | `telemetry_update`, `telemetry_stale`, `telemetry_feedback_lost` |

Both are **explanatory only** — see [rt_authority_model_v1.md](rt_authority_model_v1.md).

Telemetry bundle may embed `adapter_health` and `entity_pose_mirror`; these do not overwrite registry authority.

---

## 3. Stale vs mismatch

| Condition | Domain | Meaning | Blocks commands? |
|-----------|--------|---------|------------------|
| **Drift stale** | G3 pose sync | `‖command − feedback‖ > pose_sync_drift_threshold_m` → `SYNC_STALE` / `sync_health: stale` | Yes on spawn/move |
| **Clock-age stale** | G4 telemetry | Poll timestamp age > `telemetry_stale_s` → `telemetry_health: stale` | No |
| **Clock-age feedback loss** | G3 | Poll timestamp age > `adapter_feedback_stale_s` via `check_feedback_stale()` / `is_poll_stale()` | Explanatory helper; checked at poll time |
| **Mismatch** | G3 | Unknown entity or `sim_entity_ref` mismatch → `SYNC_MISMATCH` | Yes on spawn/move |
| **IPC loss** | G3/G4 | Worker unreachable → `adapter_feedback_lost` / `telemetry_feedback_lost` | Depends on path |

**Naming note:** G3 uses `sync_health: stale` for **spatial drift**; G4 uses `telemetry_health: stale` for **clock age**. Same word, different semantics — do not conflate.

Shared UTC helpers: `time_utils.is_poll_stale()`.

---

## 4. Telemetry stores (three-store model)

| Store | Module | Role |
|-------|--------|------|
| `TelemetryMirror` | `telemetry_bridge.py` | Last adapter poll snapshot — authoritative for S4 adapter-fed channels |
| `TelemetrySubscriptionStore` | `telemetry_subscriptions.py` | Per-session ring buffer of published channel events |
| `TelemetryBuffer` | `telemetry.py` | Legacy internal heartbeats — **not** the subscribe_telemetry path |

Consolidation boundary: adapter poll tick → mirror update → `publish_all_telemetry_channels` → subscription ring.

---

## 5. Poll ownership after R1b

```
session_manager
  → run_adapter_poll_tick (feedback? telemetry?)
  → _apply_adapter_poll_result (audit + channel publish)
  → _publish_channels_for_transition (skip_adapter_poll when entity sync already polled)
```

Manual subcommands (`adapter_poll_feedback`, `adapter_poll_telemetry`) route through the same tick — see [rt_runtime_subcommand_registry_v1.md](rt_runtime_subcommand_registry_v1.md).

Poll ticks may advance adapter `sync_seq` / `telemetry_seq` independently of bridge `world.revision` bumps between commands. Revision hint divergence policy: [rt_world_revision_hint_policy_v1.md](rt_world_revision_hint_policy_v1.md).

### Template resync (PLAT-RT-R2d)

Template apply and workflow template/reset steps call `run_template_adapter_resync()` — see [rt_template_resync_policy_v1.md](rt_template_resync_policy_v1.md). Distinct from per-entity post-spawn sync.

### PLAT-RT-G6 additions

- `check_feedback_stale()` wired in `adapter_poll` → `ADAPTER_FEEDBACK_LOST` audit (explanatory; non-blocking by default).
- Registry entities missing from feedback → `SYNC_MISMATCH` with `missing_feedback_entity`.
- `world_summary` may include `apply_lag_ms`, `last_command_utc`, `adapter_mode` — see [rt_adapter_live_sync_v1.md](rt_adapter_live_sync_v1.md).

---

## Related

- [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md)
- [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md)
- [rt_revision_vocabulary_v1.md](rt_revision_vocabulary_v1.md)
- [rt_authority_model_v1.md](rt_authority_model_v1.md)
