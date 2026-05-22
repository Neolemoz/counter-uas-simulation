# Scenario Authoring Manifest (`scenario_authoring_manifest_v1`)

**Phase:** PLAN-SA-A1 — authoring workstation foundations  
**Authority:** [AGENTS.md](../../AGENTS.md); [sa_a1_authoring_workstation_foundations_plan.md](../platform/sa_a1_authoring_workstation_foundations_plan.md)

Additive sidecar describing **authoring workflow state** for a `scenario_topology_v1` pack. **Not** parser-visible. **Not** bundled into `replay_sa_bundle_v1` as authority. **Not** a substitute for `metadata.json` topology semantics.

---

## File location

```
fixtures/scenarios/<pack_id>/authoring_manifest.json
```

One manifest per pack directory. Optional until PLAT-SA-A1 implementation; when present, must conform to this schema.

---

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | Must be `scenario_authoring_manifest_v1` |
| `schema_version` | string | Must be `1` |
| `pack_id` | string | Matches directory name and `metadata.json` scenario pack id |
| `promotion_status` | string | See enum below |
| `governance` | object | `notice` (string), `anti_claims` (string array) |

## Recommended fields

| Field | Type | Description |
|-------|------|-------------|
| `parent_pack_id` | string | Immediate derivation parent in authoring graph |
| `topology_variant_class` | string | Controlled vocabulary for variant kind (see below) |
| `validation_snapshot_ref` | string | Repo-relative path to `experiment_validation_mirror_v1` or inline snapshot id |
| `validation_pack_fingerprint` | string | SHA-256 of canonical pack file set at last validate (PLAT-SA-A1) |
| `authoring_notes` | string | Maintainer-only free text; never shown as operational guidance |
| `promotion_lineage` | array | Ordered promotion events (see below) |
| `orchestration_handoff_refs` | array | Optional manifest/queue mirror ids (explanatory) |
| `catalog_entry_id` | string | Set after `sync_sa_catalog.py` when indexed |
| `updated_at` | string | ISO-8601 UTC timestamp of last CLI promotion write |

---

## `promotion_status` enum

Transitions are **CLI-only** (`promote_scenario_pack.py` in PLAT-SA-A1). The browser never mutates status.

| Status | Meaning |
|--------|---------|
| `draft` | Pack edited; lint not run or failed since last edit |
| `linted` | `lint_scenario_pack` passed; full validate not recorded |
| `validated` | `validate_scenario.py` passed; `validation_snapshot_ref` current |
| `promoted` | Maintainer promoted; eligible for catalog sync policy |
| `orchestration_ready` | Promoted plus orchestration handoff refs recorded |
| `deprecated` | Superseded pack; still inspectable; read-only promotion semantics (PLAT-SA-A2) |
| `archived` | Terminal retired pack; no forward promote without explicit maintainer action (PLAT-SA-A2) |

**Forward transitions (typical):** `draft` → `linted` → `validated` → `promoted` → `orchestration_ready`

**Retirement (PLAT-SA-A2):** `promoted` → `deprecated` → `archived` (CLI-only; viewer read-only)

**Coarse groups:** See [scenario_authoring_operations_v1.md](scenario_authoring_operations_v1.md).

**Stale downgrade:** If pack files change after `validation_pack_fingerprint` was recorded, status MUST downgrade to `linted` or `draft` until re-validated (see [scenario_authoring_workflow_v1.md](scenario_authoring_workflow_v1.md)). Does not auto-change `deprecated` / `archived`.

## Optional fields (PLAT-SA-A2)

| Field | Type | Description |
|-------|------|-------------|
| `promotion_summary_ref` | string | Repo-relative path to `authoring_promotion_summary_v1.json` (last promote) |
| `integrity_summary_ref` | string | Repo-relative path to last integrity audit snapshot for pack |

---

## `topology_variant_class` (controlled vocabulary)

| Value | Use when |
|-------|----------|
| `baseline` | Library pack with no parent variant |
| `sensor_layout_experiment` | Sensor position/count variant |
| `topology_layout_experiment` | Zone/entity geometry variant |
| `ingress_timing_experiment` | Ingress archetype timing variant |
| `synthetic_derivation` | Tool-generated from baseline (e.g. B2 regen) |
| `maintainer_fork` | Manual fork not fitting other classes |

Unknown values: warning at manifest lint; error in `--strict` mode (PLAT-SA-A1).

---

## `parent_pack_id` vs `metadata.provenance.baseline_pack_id`

| Field | Location | Semantics |
|-------|----------|-----------|
| `parent_pack_id` | authoring manifest | **Authoring graph** — immediate parent used to create this pack |
| `baseline_pack_id` | `metadata.provenance` | **Topology semantics** — canonical comparison baseline (C1b) |

They may differ: e.g. parent = `valley_ingress_extra_valley_sensor`, baseline = `valley_ingress`. Lint SHOULD warn if `parent_pack_id` is set but `baseline_pack_id` is missing on topology experiments.

---

## `promotion_lineage` event object

| Field | Type | Description |
|-------|------|-------------|
| `event_id` | string | Stable id (e.g. `promote-2026-05-21T12:00:00Z`) |
| `from_status` | string | Prior `promotion_status` |
| `to_status` | string | New `promotion_status` |
| `actor` | string | `cli:promote_scenario_pack` or maintainer id |
| `notes` | string | Optional |

Append-only at promotion time. Viewer displays read-only.

---

## `orchestration_handoff_refs` object (optional element)

| Field | Type | Description |
|-------|------|-------------|
| `manifest_id` | string | `experiment_job_manifest_v1.manifest_id` |
| `job_id` | string | Job within manifest |
| `queue_mirror_id` | string | Frozen queue snapshot id (explanatory) |

Presence does **not** authorize execution from the browser.

---

## Example

```json
{
  "artifact_type": "scenario_authoring_manifest_v1",
  "schema_version": "1",
  "pack_id": "valley_ingress_radar_shifted_north",
  "parent_pack_id": "valley_ingress",
  "topology_variant_class": "sensor_layout_experiment",
  "promotion_status": "validated",
  "validation_snapshot_ref": "fixtures/orchestration/validation_mirrors/valley_ingress_radar_shifted_north_validation_mirror.json",
  "authoring_notes": "Radar entity shifted north for LOS masking experiment.",
  "promotion_lineage": [],
  "governance": {
    "notice": "Authoring manifest for fixture topology workflow only.",
    "anti_claims": [
      "not operational deployment state",
      "not simulation launch authority",
      "not parser contract"
    ]
  }
}
```

---

## Lint and authority (PLAT-SA-A1)

| Tool | Role |
|------|------|
| `lint_scenario_authoring_manifest.py` | Schema, enum, parent cycle detection, fingerprint consistency |
| `promote_scenario_pack.py` | Writes manifest, appends `promotion_lineage`, updates status |
| `validate_scenario.py` | Unchanged; populates validation mirror consumed by manifest |

---

## Related

- [scenario_schema_v1.md](scenario_schema_v1.md) — `scenario_topology_v1` pack layout
- [scenario_lineage_provenance_v1.md](scenario_lineage_provenance_v1.md) — three lineage planes
- [experiment_job_manifest_v1.md](experiment_job_manifest_v1.md) — orchestration jobs
- [scenario_authoring_workflow_v1.md](scenario_authoring_workflow_v1.md) — draft → promote ladder

*End of scenario_authoring_manifest_v1 schema direction.*
