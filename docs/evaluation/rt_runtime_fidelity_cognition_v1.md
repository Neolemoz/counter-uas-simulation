# RT Runtime Fidelity Cognition Contract (`rt_runtime_fidelity_cognition_v1`)

**Phase:** PLAN-RT-F5b — fidelity cognition labels and divergence (docs); PLAT-RT-F5b implements UI  
**Authority:** [rt_runtime_fidelity_coupling_v1.md](rt_runtime_fidelity_coupling_v1.md); [rt_f5b_runtime_fidelity_coupling_plan.md](../platform/rt_f5b_runtime_fidelity_coupling_plan.md)

Normative labeling for RT workstation, telemetry metadata, and capture blocks when fidelity coupling is enabled. Cognition surfaces are **read-only** — no auto-correct of display pose or registry commands.

---

## 1. Label taxonomy

| Label | Use on | Meaning |
|-------|--------|---------|
| `command_authoritative` | Registry poses, capture `command_pose`, spawn/move | Bridge command truth |
| `truth_attested` | Sim pose, AGL, LOS/dome truth snapshots | Sim-scoped attestation when `enable_fidelity_coupling=true` |
| `explanatory` | F4 LOS/dome heuristics, contours, recommendations, sync drift | Heuristic or mirror — not operational truth |

Every truth-attested UI strip **must** show governance banner from [rt_runtime_fidelity_coupling_v1.md](rt_runtime_fidelity_coupling_v1.md) §1.

---

## 2. UI surfaces (PLAT advisory)

| Surface | Behavior |
|---------|----------|
| `TerrainCognitionStrip` | When coupling on: show `sim_agl_m` with `truth_attested` badge alongside `display_agl_m` with `explanatory` |
| `RuntimeCognitionHub` | Hub line: `Fidelity: truth_attested (sim)` or `Fidelity: off (stub)` |
| Cesium entity card | Dual z readout when both fictional offset and sim AGL present — never hide explanatory label |
| `BackgroundDiagnostics` | Session flag `enable_fidelity_coupling` visible to maintainer |
| Experiment workbench | F5b fidelity compare strip (P2) — read-only badges only |

Forbidden lexicon: [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) — no “detected”, “tracked”, “neutralized”, “readiness”, or operational sensor claims on truth-attested strips.

---

## 3. Stale truth handling

Reuse frozen stale semantics; F5b adds fidelity-specific audit and UI flags only.

| Condition | Domain | UI / audit |
|-----------|--------|------------|
| Spatial drift | G3 `sync_health: stale` | Existing `SYNC_STALE`; when coupling on, also `fidelity_truth_stale` if truth poll age exceeds threshold |
| Telemetry clock age | G4 `telemetry_health: stale` | Existing; truth bundle may inherit stale if embedded in telemetry poll |
| Coupling off | — | `fidelity_attestation_status: unavailable` — not an error |
| Partial entity truth | Missing `truth_attested_pose` for some entities | Per-entity flag `partial_truth`; `command_pose` still authoritative |

### Audit event (docs taxonomy)

Align with [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md):

| `event_kind` | When |
|--------------|------|
| `fidelity_truth_update` | Successful truth snapshot poll |
| `fidelity_truth_stale` | Truth attestation older than configured threshold or sync unhealthy |
| `fidelity_truth_mismatch` | Truth entity not in registry (P0) |
| `fidelity_capture_snapshot` | Capture normalized with `fidelity_pose_block` |

Audit entries are **explanatory evidence** — not authority state.

---

## 4. Divergence handling

| Divergence type | Detection | Action |
|-----------------|-----------|--------|
| `cognition_truth_divergence` | `los_cognition_label` ≠ `los_truth_label` (when both present) | Badge on UI + experiment compare; **no** auto-sync |
| Pose drift | `pose_truth_drift_m` > threshold (config in PLAT) | Surface in capture `fidelity_pose_block`; optional session flag |
| Fictional vs sim AGL | \|display_agl_m − sim_agl_m\| large | Dual labels required — do not collapse to single “AGL” |
| Intent vs truth lag | Sim poll newer than last command apply | `intent_truth_lag` informational flag |

**Normative rule:** Divergence never triggers registry rewrite, automatic capture failure, or SA import.

---

## 5. Telemetry metadata (additive)

When coupling on, optional fields on subscription events (PLAT):

| Field | Value |
|-------|-------|
| `fidelity_label` | `truth_attested` \| `explanatory` \| `command_authoritative` |
| `fidelity_attestation_status` | `available` \| `stale` \| `unavailable` |
| `governance_banner` | Fidelity banner string |

Primary `authority_label` on payload remains per [rt_authority_model_v1.md](rt_authority_model_v1.md); fidelity fields are **supplementary**.

---

## 6. Workstation integration

See [rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md). F5b adds read-only hub lines and diagnostics — no new bridge commands, no workflow phase changes.

| Banner id | When shown |
|-----------|------------|
| `BANNER_FIDELITY_TRUTH` | `enable_fidelity_coupling=true` on any panel showing truth-attested data |
| Existing T1/T3/T4/T5 banners | Unchanged — additive only |

---

## 7. Explicit non-goals

- Auto-correct display pose from sim truth
- SA replay scrubber or live RT hooks
- Tactical mode or assignment authority changes
- Blocking spawn/move on cognition/truth divergence (beyond existing G3 mismatch rules)

---

## Related

- [rt_runtime_fidelity_coupling_v1.md](rt_runtime_fidelity_coupling_v1.md)
- [rt_runtime_realism_expansion_v1.md](rt_runtime_realism_expansion_v1.md)
- [rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md)
- [rt_experiment_metrics_v1.md](rt_experiment_metrics_v1.md) §11
