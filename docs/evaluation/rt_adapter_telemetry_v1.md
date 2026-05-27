# RT Adapter Telemetry Contract (`rt_adapter_telemetry_v1`)

**Phase:** PLAT-RT-G4 — runtime telemetry bridge  
**Authority:** [rt_g4_telemetry_bridge_plan.md](../platform/rt_g4_telemetry_bridge_plan.md)

IPC telemetry bundle between bridge and **adapter_worker**. Explanatory only — not replay authority.

---

## 1. Authority separation

| Layer | Role |
|-------|------|
| `EntityRegistry` | Command-authoritative poses |
| `PoseSyncMirror` (G3) | Sync health, `SYNC_STALE` |
| `TelemetryMirror` (G4) | Telemetry channel snapshots only |

---

## 2. `poll_telemetry`

**Request payload:** `{}` or test / fidelity fields:

| Field | Description |
|-------|-------------|
| `mock_stale_telemetry` | If true, return stale `timestamp_utc` (test only) |
| `enable_fidelity_coupling` | When true (and worker attached with coupling), include `fidelity_truth` (PLAT-RT-F5b) |
| `fidelity_ground_z_m` | Ground reference for mock `sim_agl_m` (default 0) |

**Response:** `rt_adapter_telemetry_v1`

```json
{
  "schema": "rt_adapter_telemetry_v1",
  "timestamp_utc": "2026-05-22T12:00:00+00:00",
  "telemetry_seq": 4,
  "clock_mirror": { "paused": false, "mode": "mock" },
  "adapter_health": { "alive": true, "entity_count": 1, "mode": "mock", "paused": false },
  "entity_pose_mirror": {
    "entities": [
      { "entity_id": "uuid", "sim_entity_ref": "sim-abc", "pose": { "x": 0, "y": 0, "z": 10 } }
    ]
  },
  "world_revision_hint": { "telemetry_seq": 4, "sync_seq": 4 },
  "fidelity_truth": {
    "schema": "rt_fidelity_truth_snapshot_v1",
    "session_id": "sess-uuid",
    "timestamp_utc": "2026-05-22T12:00:00+00:00",
    "governance_banner": "RT FIDELITY TRUTH — sim-scoped attestation only; not SA replay or operational sensor authority",
    "attestation_status": "available",
    "entity_truth": [],
    "los_truth": { "label": "clear", "pair_entity_ids": [] },
    "dome_truth": { "sensor_id": "radar_north", "entities_in_nominal_dome": 0 }
  }
}
```

`fidelity_truth` is **optional** — present only when `enable_fidelity_coupling=true` on bridge config and poll request. Sim-scoped attestation only — never merged into `EntityRegistry`. See [rt_runtime_fidelity_coupling_v1.md](rt_runtime_fidelity_coupling_v1.md).

`world_revision_hint` is an adapter poll-generation bundle — **not** a copy of bridge `world.revision`. Numeric divergence from `world.revision` is expected under load. See [rt_world_revision_hint_policy_v1.md](rt_world_revision_hint_policy_v1.md).

---

## 3. Telemetry mirror policy

- `telemetry_revision` tracks `telemetry_seq` from adapter poll.
- Stale when poll age > `telemetry_stale_s` → `telemetry_health: stale` (no command failure).
- IPC failure → `telemetry_feedback_lost` audit.
- `world_revision_hint` is stored in `TelemetryMirror` for capture normalization; not merged into `world_summary` telemetry channel payload.

---

## 4. Channel sources (adapter on)

| Channel | Source |
|---------|--------|
| `entity_pose_mirror` | Adapter feedback poses |
| `clock_mirror` | Worker sim clock |
| `session_health` | Bridge state + adapter_health |
| `world_summary` | Registry + telemetry_revision/health |
| `lifecycle_state` | Bridge only |

---

## 5. Telemetry stores (consolidation boundary)

| Store | Role |
|-------|------|
| `TelemetryMirror` | Last adapter poll — authoritative for adapter-fed S4 channels |
| `TelemetrySubscriptionStore` | Ring buffer of published subscription events |
| `TelemetryBuffer` | Legacy internal heartbeats only — **not** `subscribe_telemetry` |

Update path after PLAT-RT-R1b: `run_adapter_poll_tick` → mirror update → `publish_all_telemetry_channels` → subscription ring. See [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md).

---

## Related

- [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md) — pose sync (G3)
- [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md) §4
- [rt_authority_model_v1.md](rt_authority_model_v1.md)
- [rt_revision_vocabulary_v1.md](rt_revision_vocabulary_v1.md)
