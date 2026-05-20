# Comparison Foundations (D1 implemented)

Additive comparison contracts for replay experimentation. **Explanatory comparability aids only** — not rankings, readiness, or operational effectiveness.

Implementation freeze: [sa_d1_comparative_replay_freeze_audit.md](sa_d1_comparative_replay_freeze_audit.md). Integrity audits: [sa_stabilization_plan.md](sa_stabilization_plan.md).

See also: [paired_seed_delta.md](demo_cases/paired_seed_delta.md), [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md), [replay_demo_review_workflow_r1.md](replay_demo_review_workflow_r1.md).

## Identifier normalization

| Identifier | Role | Example |
|------------|------|---------|
| `pack_id` | Canonical catalog key; directory under `fixtures/scenarios/` | `saturation_ingress` |
| `scenario_id` | Stable metadata id (may differ from pack_id) | `saturation_ingress_demo` |
| `catalog_entry_id` | Same as `pack_id` in `comparison_hints` | `saturation_ingress` |
| `topology_key` | Topology comparison anchor (future D1) | `saturation_ingress` |
| `demo_bundle_url` | Viewer load path | `/demo/saturation_ingress/index.json` |
| `demo_bundle` | Repo-relative packed bundle path | `fixtures/sa_r0/demo_saturation_ingress/index.json` |

Lint enforces catalog `pack_id` matches scenario pack directory name.

## Comparison modes (planned)

### 1. Matched-seed replay A/B

**Existing anchor:** `scripts/evaluation/replay_observability.py paired-comparison` → `matched_seed_comparison_report`.

**Hooks:** `bundle.lineage.seed`, `bundle.lineage.cohort`, `comparison_hints.seed`.

Use when comparing two runs under the same seed and cohort — parser-visible deltas remain authoritative; replay bundles are explanatory overlays.

### 2. Topology A/B (frozen — PLAT-SA-D1)

**Hook:** `comparison_hints.topology_key` (= `pack_id`).

Use when comparing replay behavior across different scenario packs. SA-R0 viewer compare mode loads two bundles side-by-side; sensor studies may share log via `shared_log_ref` in `compare_pairs_v1.json`.

### 3. Sensor placement study (D1)

**Hook:** `comparison_hints.sensor_layout_id` (derived from `topology.json` entity positions hash via `compute_sensor_layout_id`).

Use when varying static site geometry within the same ingress archetype. Experiment packs under `fixtures/scenarios/valley_ingress_*` share the valley demo log.

### 4. Monte Carlo topology sweep

**Hooks:** `comparison_hints.sweep_id`, `lineage.seed`, MC aggregate scripts in `scripts/evaluation/README.md`.

Use when aggregating many seeded runs — not for single-replay tactical claims.

## `comparison_hints` bundle block (C1b)

Optional on `replay_sa_bundle_v1`:

```json
{
  "topology_key": "saturation_ingress",
  "scenario_id": "saturation_ingress_demo",
  "catalog_entry_id": "saturation_ingress",
  "duration_class": "medium",
  "seed": 101,
  "comparison_ready": true
}
```

Packed by `replay_sa_bundle.py` at bundle build time. Viewer does **not** implement compare UI in C1b.

## Governance dos and don'ts

**Do:**

- Treat comparison hooks as catalog and provenance aids for reviewers.
- Keep matched-seed pairing explicit in lineage metadata.
- Document fictional topology caveats when comparing geometry-heavy scenarios.

**Don't:**

- Frame comparison outputs as operational effectiveness or deployment planning.
- Use `ambiguity_profile.level` or `narrative_rank` as severity or readiness labels.
- Imply real geography from `georef_display.anchor`.

## D1 implemented (PLAT-SA-D1)

- Side-by-side compare viewer (`?pair=`, `?compare=packA,packB`) — see [replay_compare_v1.md](replay_compare_v1.md)
- Topology diff panel + map delta highlighting
- Replay outcome comparison panel (explanatory observations)
- Sensor placement experiment packs + `compare_pairs_v1.json`

## D2 implemented (PLAT-SA-D2)

- `replay_mc_sweep_v1` manifests under `fixtures/sa_r0/sweeps/` — see [replay_mc_sweep_v1.md](replay_mc_sweep_v1.md)
- Sweeps catalog index `fixtures/scenarios/sweeps_index_v1.json` — viewer `?sweep=` picker
- Spatial analytics overlays — see [replay_spatial_analytics_v1.md](replay_spatial_analytics_v1.md)
- Static `matched_seed_comparison_report` fixture optional per sweep (explanatory panel)
- `export_replay_analytics_report.py` → `replay_compare_report_v1.json`

## D3 implemented (PLAT-SA-D3)

- Sweep narrative intelligence + replay pattern taxonomy — see [replay_narrative_intelligence_v1.md](replay_narrative_intelligence_v1.md)
- Sweep workstation cohort navigation + N-slot filmstrip (2–4 members)
- Pack-time overlay `active_t_range` vs log span lint (warnings)

## D3 follow-up

- Live MC cohort import from `runs/logs` (CLI only, no viewer WebSocket)
