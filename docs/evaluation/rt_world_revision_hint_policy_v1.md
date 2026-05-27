# RT World Revision Hint Policy (`rt_world_revision_hint_policy_v1`)

**Phase:** PLAT-RT-R3d — world revision hint divergence policy  
**Authority:** [rt_r1_architecture_stabilization_review_r1.md](rt_r1_architecture_stabilization_review_r1.md) finding R1-SYNC-05; [rt_revision_vocabulary_v1.md](rt_revision_vocabulary_v1.md)

Closes ambiguity around revision counter ownership, valid divergence, and stale vs mismatch semantics. **Mirrors remain non-authoritative.**

---

## 1. Counter ownership and update timing

| Counter | Owner module | Bumps when | Authority |
|---------|--------------|------------|-----------|
| `world.revision` | `world_state.py` | `spawn_entity`, `move_entity`, `delete_entity`, template apply, `reset_session` | **Command-authoritative** |
| `bridge_revision` | IPC alias | Same instant as `world.revision` on entity commands | Command-authoritative (field name in adapter IPC) |
| `EntitySyncEntry.sync_revision` | `pose_sync.py` | Entity command recorded in mirror | Explanatory snapshot of `world.revision` at command time |
| `world_summary.sync_revision` | `pose_sync.summary(world.revision)` | Included on each `world_summary` build | **Current** `world.revision` (summary field name — not per-entity) |
| `sync_seq` / `last_sync_seq` | `adapter_worker.py` / `PoseSyncMirror` | Each `poll_feedback` | Explanatory adapter feedback generation |
| `telemetry_seq` | `adapter_worker.py` | Each `poll_telemetry` | Explanatory adapter telemetry generation |
| `telemetry_revision` | `telemetry_bridge.py` | Copy of last poll `telemetry_seq` | Explanatory |
| `world_revision_hint` | `adapter_worker.py` → `TelemetryMirror` | Each `poll_telemetry` | Explanatory `{telemetry_seq, sync_seq}` bundle |
| `registry_revision` | `capture_normalize.py` | From snapshot entity state at capture | Capture-scoped explanatory |
| `conversion_revision` | `capture_normalize.py` | Normalization / re-normalize | Staging-scoped pipeline version |

See [rt_revision_vocabulary_v1.md](rt_revision_vocabulary_v1.md) for lifecycle reset rules.

---

## 2. Naming clarification

Two distinct uses of `sync_revision`:

| Context | Meaning |
|---------|---------|
| `EntitySyncEntry.sync_revision` | Per-entity snapshot of `world.revision` when command was recorded |
| `world_summary.sync_revision` | Live `world.revision` passed into `pose_sync.summary()` — equals `world_summary.revision` |

Do not compare per-entity `sync_revision` to `world_revision_hint.sync_seq` for command failure — different domains.

---

## 3. Valid divergence

Expected and **not** a sync failure:

| Scenario | Example | Interpretation |
|----------|---------|----------------|
| Hint seqs ≠ `world.revision` | `world.revision=5`, hint `{sync_seq:3, telemetry_seq:4}` | Different counter domains — adapter polls lag command bumps |
| `telemetry_revision` ≠ `world.revision` | After several commands between telemetry polls | Poll generation counter, not registry revision |
| Per-entity `sync_revision` < current `world.revision` | Entity not moved since spawn | Stale command-time snapshot in mirror entry |
| Hint advances without registry change | Telemetry poll without entity command | Normal poll activity |
| `conversion_revision` ≠ `world.revision` | Capture normalized after session | Staging pipeline scope only |

**Rule:** Numeric mismatch between explanatory counters and `world.revision` is expected under load. Reviewers must not treat hint or telemetry revision as command truth.

---

## 4. Invalid divergence (policy violations)

| Violation | Why forbidden |
|-----------|---------------|
| Using `world_revision_hint` or `telemetry_revision` as command authority | Violates [rt_authority_model_v1.md](rt_authority_model_v1.md) |
| Comparing hint to `world.revision` to emit `SYNC_STALE` / `SYNC_MISMATCH` | Stale/mismatch uses pose drift and ref map — see §5 |
| Promoting sim/telemetry mirror pose over registry on divergence | G3/G4 mirrors are explanatory |
| Using `conversion_revision` as live session revision | Capture-scoped only |
| Expecting `world_revision_hint` to contain `world.revision` | Hint shape is `{telemetry_seq, sync_seq}` only |

---

## 5. Stale vs mismatch (revision context)

Revision counters **do not** drive stale/mismatch detection. Detection uses pose mirror state:

| Condition | Error | Audit | Revision fields involved |
|-----------|-------|-------|------------------------|
| Pose drift > threshold | `SYNC_STALE` | `sync_stale` | Per-entity command vs feedback pose; `sync_seq` updated on poll |
| Unknown entity / ref mismatch | `SYNC_MISMATCH` | `sync_mismatch` | Ref map inconsistency |
| Feedback IPC failure / poll age | `ADAPTER_FEEDBACK_LOST` | `adapter_feedback_lost` | `last_sync_seq` may be stale |
| OK feedback poll | — | `sync_update` | `sync_seq` increments |
| Telemetry poll age > threshold | — (explanatory) | `telemetry_stale` | `telemetry_revision` unchanged semantics |
| OK telemetry poll | — | `telemetry_update` | `telemetry_revision` ← `telemetry_seq`; hint updated |

Telemetry stale is **explanatory only** — does not fail bridge commands. See [rt_adapter_feedback_v1.md](rt_adapter_feedback_v1.md), [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md).

---

## 6. Surface exposure

| Surface | `world.revision` | `world_revision_hint` | Notes |
|---------|------------------|----------------------|-------|
| `world_summary` telemetry channel | `revision`, `sync_revision` | **Not included** | Adds `telemetry_revision`, `telemetry_health` when adapter active |
| `TelemetryMirror.summary()` | — | Included | Used for capture normalization |
| Normalized `telemetry_health_summary` | — | Included when mirror present | Explanatory capture snapshot |

---

## 7. Capture revision refs

| Field | Location | Role |
|-------|----------|------|
| `snapshot_revision` | `capture_timestamps` in normalized manifest | `world.revision` at capture snapshot |
| `registry_revision` | Per-entity in `entity_pose_history` | Entity revision at capture |
| `sync_revision` | Per-entity in `entity_pose_history` | Mirror entry command-time snapshot |
| `conversion_revision` | Normalized manifest + candidate | Normalization pipeline version |
| `world_revision_hint` | `telemetry_health_summary` | Adapter hint at capture instant |

`conversion_revision` increments on re-normalize when `input_content_hash` changes; idempotent when hash matches. See [rt_capture_normalization_v1.md](rt_capture_normalization_v1.md).

---

## 8. Audit events and revision context

| `command_type` / `event_type` | When | Revision relevance |
|-------------------------------|------|-------------------|
| `sync_update` | Successful feedback poll or resync apply | `sync_seq` may advance; not compared to `world.revision` |
| `sync_stale` | Pose drift exceeds threshold | Uses pose comparison, not hint |
| `sync_mismatch` | Ref/entity map inconsistency | Uses ref map, not hint |
| `telemetry_update` | Successful telemetry poll | Updates `telemetry_revision`, `world_revision_hint` |
| `telemetry_stale` | Telemetry poll age exceeded | Explanatory; hint unchanged semantics |
| `capture_normalized` | Normalization succeeded | Sets `conversion_revision` on manifest |
| `export_pose_normalized` | Pose cognition attached (R2e) | Includes `conversion_revision` in detail |

Export audit uses `event_type`; session audit uses `command_type`. Both are explanatory — not parser contracts.

---

## 9. Related

- [rt_revision_vocabulary_v1.md](rt_revision_vocabulary_v1.md)
- [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md)
- [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md)
- [rt_authority_model_v1.md](rt_authority_model_v1.md)
- [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md)
