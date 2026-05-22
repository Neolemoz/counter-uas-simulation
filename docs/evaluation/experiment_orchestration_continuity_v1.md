# Experiment Orchestration Continuity (v1)

**Phase:** PLAN-SA-I1 — end-to-end deterministic experimentation chain  
**Authority:** [AGENTS.md](../../AGENTS.md); [experiment_workflow_continuity_v1.md](experiment_workflow_continuity_v1.md) (H4 navigation).

Defines the **fixed artifact chain** from scenario topology through orchestration to publication. All links are **explanatory** unless noted as CLI-authoritative for that plane.

---

## 12-pack catalog coverage

All packs in `fixtures/scenarios/index.json` MUST have:

| Artifact | Path pattern |
|----------|----------------|
| Topology | `fixtures/scenarios/<pack_id>/` |
| Authoring | `fixtures/scenarios/<pack_id>/authoring_manifest.json` |
| Validation mirror | `fixtures/orchestration/validation_mirrors/<pack_id>_validation_mirror.json` |
| Job manifest | `fixtures/orchestration/manifests/<pack_id>_validation.json` (validation-only) or synthetic manifest |
| Ops sidecar | `fixtures/orchestration/ops/<manifest_id>_ops.json` |
| Queue snapshot | `fixtures/orchestration/queues/<queue_id>.json` |
| Audit report | `fixtures/orchestration/audits/<queue_id>_report.json` |

Reference full synthetic pipeline: `ridge_defense_synthetic` only.

---

## Continuity chain

```
scenario_topology_v1
  → scenario_authoring_manifest_v1
  → experiment_validation_mirror_v1
  → experiment_job_manifest_v1
  → experiment_orchestration_ops_manifest_v1
  → experiment_run_queue_v1
  → experiment_run_report_v1
  → [optional] queue_claim_token_v1
  → [optional] worker_execution_record_v1
  → [optional] experiment_orchestration_async_manifest_v1
  → replay demo bundle (fixtures/sa_r0/demo_<pack_id>/)
  → replay_corpus_index_v1 entry
  → replay_corpus_publication_packet_v1 (when in release scope)
```

---

## Cross-plane checks (I1 integrity audit)

- Authoring `orchestration_handoff_refs` ↔ orchestration index `manifest_id` / `queue_mirror_id`
- Manifest `scenario_pack_id` ↔ validation mirror `scenario_pack_id`
- Queue `manifest_ref` ↔ job manifest `manifest_id`
- Ops `manifest_fingerprint` ↔ current job manifest hash
- Queue `provenance.bundle_path` ↔ bundle `index.json` when `replay_generated`

### Async plane (PLAT-SA-I2, optional)

- `claim_token.manifest_id` ↔ job manifest `manifest_id`
- `claim_token.snapshot_hash` ↔ queue file hash
- `worker_record.claim_id` ↔ claim token `claim_id`
- `execution_fingerprint` ↔ async sidecar when async bookkeeping present
- `async_execution_status` = `quarantined` blocks I1 forward promote (CLI guard)

---

## Viewer URL hooks (read-only)

| Param | Segment | Purpose |
|-------|---------|---------|
| `demo` / `authoring_pack` | scenario | Authoring mirror |
| `orchestration_queue` | scenario, replay, corpus | Queue snapshot |
| `orchestration_manifest` | scenario | Ops sidecar (explanatory) |
| `corpus_entry` | corpus | Corpus browser |

CLI owns all transitions. Browser never launches `run_experiment_queue.py`.

---

## Related

- [authoring_workflow_continuity_v1.md](authoring_workflow_continuity_v1.md)
- [experiment_workflow_scenario_to_replay_v1.md](experiment_workflow_scenario_to_replay_v1.md)
- [experiment_orchestration_operations_v1.md](experiment_orchestration_operations_v1.md)

*End of experiment orchestration continuity v1.*
