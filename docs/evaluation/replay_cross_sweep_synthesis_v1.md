# Cross-Sweep Replay Synthesis (`cross_sweep_synthesis_v1`)

Normative semantics for deterministic rollups across replay MC sweep families. **Explanatory replay interpretation only** — not tactical advice or operational planning.

See also: [replay_mc_sweep_v1.md](replay_mc_sweep_v1.md), [replay_pattern_taxonomy_v1.md](replay_pattern_taxonomy_v1.md), [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md).

## Artifact placement

| Location | Notes |
|----------|-------|
| `fixtures/sa_r0/synthesis/cross_sweep_synthesis_v1.json` | Canonical corpus rollup |
| `platform/sa-r0-viewer/public/demo/synthesis/` | Viewer mirror |

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `cross_sweep_synthesis_v1` |
| `schema_version` | yes | `cross_sweep_synthesis_v1` |
| `governance` | yes | Notice + anti-claims |
| `sweep_ids` | yes | Ordered sweep IDs included |
| `pattern_frequency_rollup` | yes | Cross-sweep pattern counts |
| `ambiguity_concentration_comparison` | yes | Top cells + overlap |
| `topology_sensitivity_rollup` | yes | Per-sweep + shared topology keys |
| `los_instability_rollup` | yes | Sweep rankings + LOS concentration |
| `divergence_rollup` | yes | Cross-sweep divergence incidence |
| `cognition_rollup` | no | Embedded cognition bullets (see replay_cognition_rollup_v1) |
| `interpretation_caveats` | yes | Mandatory reviewer caveats |

## Rollup blocks

### `pattern_frequency_rollup`

| Field | Notes |
|-------|-------|
| `by_pattern` | Map pattern_id → `{ count, sweep_ids[], member_count }` |
| `dominant_across_corpus` | Ordered pattern IDs by total member count |

### `ambiguity_concentration_comparison`

| Field | Notes |
|-------|-------|
| `top_cells_by_sweep` | Map sweep_id → top cell indices + counts |
| `shared_hotspot_cells` | Cell indices appearing in top-N of ≥2 sweeps |

### `topology_sensitivity_rollup`

| Field | Notes |
|-------|-------|
| `by_sweep` | Map sweep_id → `{ max_count, median_count, topology_keys[] }` |
| `shared_topology_keys` | Keys appearing in ≥2 sweep linkage blocks |

### `los_instability_rollup`

| Field | Notes |
|-------|-------|
| `by_sweep` | Map sweep_id → `{ max_los_count, member_los_totals[] }` |
| `ranked_sweep_ids` | Sweeps ordered by max LOS layer count (desc) |

### `divergence_rollup`

| Field | Notes |
|-------|-------|
| `sweeps_with_divergence` | Sweep IDs with ≥1 member divergence flag |
| `member_divergence_count` | Total members with topology-sensitive divergence tag |

## Governance

**Do:** Keep rollups replay-local; label as explanatory concentration summaries.

**Don't:** Imply validated probability, deployment confidence, or tactical superiority.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_cross_sweep_synthesis.py`, `gen_e2_research_fixtures.py` |
| Consumer | Viewer synthesis panel, publication exports, research bundles |
