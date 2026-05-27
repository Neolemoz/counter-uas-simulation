# RT Runtime Fidelity Coupling Contract (`rt_runtime_fidelity_coupling_v1`)

**Phase:** PLAN-RT-F5b — runtime fidelity coupling (docs); PLAT-RT-F5b implements  
**Prerequisite:** PLAT-RT-G3/G6, PLAT-RT-R2e, PLAT-RT-F4, PLAT-RT-F5 frozen  
**Authority:** [rt_f5b_runtime_fidelity_coupling_plan.md](../platform/rt_f5b_runtime_fidelity_coupling_plan.md)

Defines **default-off** coupling between RT sandbox runtime and Gazebo / sim-scoped sensor truth. Coupled truth is **sim-scoped attestation** for RT review and experiment metrics — not SA replay authority, not operational sensor truth, not registry command authority.

---

## 1. Governance

| Rule | Detail |
|------|--------|
| Feature flag | `enable_fidelity_coupling` default `false` on session or adapter config |
| Banner | `RT FIDELITY TRUTH — sim-scoped attestation only; not SA replay or operational sensor authority` |
| Scope | RT sandbox only — no `platform/sa-r0-viewer/` changes |
| Registry | `EntityRegistry` remains `command_authoritative` — sim truth never overwrites spawn/move or normalized `command_pose` |
| F4 boundary | Fictional terrain / heuristic LOS from [rt_runtime_realism_expansion_v1.md](rt_runtime_realism_expansion_v1.md) remain **explanatory** unless a truth channel is present and coupling is on |

When `enable_fidelity_coupling=false` (default): all F5b truth fields are omitted or `null`; UI hides truth-attested labels; metrics fidelity report returns `fidelity_attestation_status: unavailable`.

---

## 2. Authority stack (normative)

Three layers — see also [rt_runtime_fidelity_cognition_v1.md](rt_runtime_fidelity_cognition_v1.md).

| Layer | Label | Source | Overwrites registry? |
|-------|-------|--------|----------------------|
| Command | `command_authoritative` | `EntityRegistry`, bridge commands, `snapshot.json` / `command_pose` | N/A (authority) |
| Truth-attested | `truth_attested` | Adapter poll when coupling on: `entity_state`, optional `fidelity_truth` | **Never** |
| Explanatory | `explanatory` | F4/V2 heuristics, `PoseSyncMirror` drift evidence, recommendations | **Never** |

**Session sim truth:** last adapter-attested sim snapshot for the active `session_id` (ephemeral).  
**Command intent:** accepted bridge command and registry pose at command time — authoritative for capture export per [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md).

---

## 3. Gazebo fidelity coupling

### 3.1 Runtime entity ↔ Gazebo pose sync

Aligns with [rt_adapter_live_sync_v1.md](rt_adapter_live_sync_v1.md) §1:

| Surface | Role | F5b label |
|---------|------|-----------|
| `EntityRegistry` | Command-authoritative poses | `command_authoritative` |
| `entity_pose_cmd` | Bridge→sim **command intent** (transient) | intent path |
| `entity_state` | Sim→adapter feedback at poll instant | **session sim truth** when coupling on |
| `PoseSyncMirror` | Drift/stale/mismatch diagnostics | `explanatory_sync`; pose comparison may surface `truth_attested` sub-label when coupling on |

**Normative rule:** `feedback_pose` / `entity_state` poses are truth-attested for **comparison and cognition only**. Divergence from `command_pose` is **explanatory-only mismatch** — same rule as R2e; F5b does not add merge-into-registry behavior.

### 3.2 Terrain height / AGL coupling

| Mode | `registry_z` | Display / cognition |
|------|--------------|---------------------|
| F4/V2 default | command truth | `display_agl_m` from fictional heightmap ([rt_v2_terrain_realism_v1.md](rt_v2_terrain_realism_v1.md)) |
| F5b coupled | still command truth | `sim_ground_z_m` from flat Gazebo ground (default) or future sim ground sample; `sim_agl_m = sim_pose.z - sim_ground_z_m` |
| Dual display | unchanged registry commands | When coupling on, UI **must** show both `display_agl_m` (explanatory) and `sim_agl_m` (truth-attested) when both available |

**Forbidden:** Rewriting registry `z` from sim pose, fictional terrain sample, or AGL-derived correction.

### 3.3 Session truth vs command intent

| Scenario | Interpretation | Blocks commands? |
|----------|----------------|------------------|
| Fresh command, stale sim feedback | G3 `sync_health: stale` (spatial drift) | Existing G3 rules — unchanged |
| Stale command age, fresh sim poll | Truth snapshot newer than last command apply — surface `intent_truth_lag` flag (informational) | No new blocking in PLAN |
| Adapter off / stub | No session sim truth — F4-only path | N/A |
| Capture instant | `command_pose` authoritative; optional `truth_attested_pose` in `fidelity_pose_block` | Normalization must not fail on divergence |

Stale vocabulary reuses [rt_poll_sync_semantics_v1.md](rt_poll_sync_semantics_v1.md) — do not conflate G3 drift stale with G4 telemetry clock stale.

### 3.4 Capture: `fidelity_pose_block` (additive)

Optional block on `rt_normalized_capture_v1` (PLAT-RT-F5b P0). Extends tri-source pose cognition to quad-source:

```json
{
  "fidelity_pose_block": {
    "schema": "rt_fidelity_pose_block_v1",
    "governance_banner": "RT FIDELITY TRUTH — sim-scoped attestation only; not SA replay or operational sensor authority",
    "enable_fidelity_coupling": true,
    "adapter_attached": true,
    "per_entity": [
      {
        "entity_id": "uuid",
        "command_pose": { "x": 0, "y": 0, "z": 10, "yaw_deg": 0 },
        "truth_attested_pose": { "x": 0.1, "y": 0, "z": 10.2, "yaw_deg": 0 },
        "pose_truth_drift_m": 0.22,
        "sim_agl_m": 10.2,
        "flags": ["cognition_truth_divergence"]
      }
    ]
  }
}
```

| Field | Rule |
|-------|------|
| `truth_attested_pose` | From `entity_state` / feedback at capture instant |
| `pose_truth_drift_m` | Euclidean ‖command − truth‖; explanatory |
| `sim_agl_m` | Only when ground sample available |

---

## 4. Sensor-truth coupling (default-off)

Optional read-only bundles when `enable_fidelity_coupling=true`. **Not** SA parser truth.

### 4.1 `rt_fidelity_truth_snapshot_v1` (adapter / capture)

```json
{
  "schema": "rt_fidelity_truth_snapshot_v1",
  "session_id": "sess-00000001",
  "timestamp_utc": "2026-05-26T15:00:00+00:00",
  "governance_banner": "RT FIDELITY TRUTH — sim-scoped attestation only; not SA replay or operational sensor authority",
  "attestation_status": "available",
  "visibility_truth": { "ref": "sim_visibility_bundle_v1", "label": "clear" },
  "los_truth": { "label": "terrain_blocked", "pair_entity_ids": ["a", "b"] },
  "dome_truth": { "sensor_id": "radar_north", "entities_in_nominal_dome": 2 },
  "occlusion_truth": { "occluder_ids": ["ridge_sim_01"], "optional": true }
}
```

| Channel | PLAT source (advisory) | vs F4 explanatory |
|---------|------------------------|-------------------|
| Visibility truth | Sim or allow-listed ROS read via adapter | `visibility_context` / manifest heuristics stay explanatory |
| LOS truth | Sim ray / occlusion query | `los_cognition_label` unchanged; add `los_truth_label` only when coupling on |
| Dome truth | Sim sensor footprint config | V2 “nominal dome — not coverage proof” for UI-only layers preserved |
| Env occlusion truth | Static sim occluders | F4 `occlusion_markers` remain fictional fixtures |

`attestation_status`: `available` \| `stale` \| `unavailable`.

### 4.2 IPC placement (PLAT advisory)

Prefer additive optional field `fidelity_truth` on [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md) poll response and in `NormalizationContext` at capture — **not** [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) HTTP changes.

New ROS topics beyond [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md) allow-list are **out of scope** for PLAT-RT-F5b unless a future wave re-opens boundary audit.

---

## 5. Multi-session isolation

Truth snapshots are scoped to `session_id`. No cross-session truth merge or compare authority — see [rt_multi_session_registry_v1.md](rt_multi_session_registry_v1.md).

---

## 6. Export and SA boundary

Fidelity blocks on normalized capture are `replay_boundary_scoped` — same as [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md). SA import remains manual (SA1). Truth fields in experiment metrics are **derived mirrors** — not handoff auto-approval.

---

## 7. Explicit non-goals

- Bridge HTTP protocol changes
- Parser/topic/schema changes
- SA viewer or replay bundle schema changes in PLAN wave
- Tactical controller redesign
- Distributed M3
- Physics or sensor model redesign
- Operational readiness or effectiveness claims

---

## Related

- [rt_runtime_fidelity_cognition_v1.md](rt_runtime_fidelity_cognition_v1.md)
- [rt_authority_model_v1.md](rt_authority_model_v1.md)
- [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md)
- [rt_adapter_live_sync_v1.md](rt_adapter_live_sync_v1.md)
- [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md) §11
