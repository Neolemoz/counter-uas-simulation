# Replay Pattern Taxonomy (`replay_pattern_taxonomy_v1`)

Deterministic, replay-local pattern tags for sweep member classification and cohort grouping. **Not** parser failure classes (F1–F5), realism stressors (R0–R5), or operational doctrine.

See also: [replay_narrative_intelligence_v1.md](replay_narrative_intelligence_v1.md), [classify_selection_oracle_divergence.py](../../scripts/evaluation/classify_selection_oracle_divergence.py) (separate D0–D5 eval taxonomy).

## Pattern IDs

| `pattern_id` | Replay-local meaning |
|--------------|----------------------|
| `los_fragmented_replay` | LOS degradation segments concentrate across replay; terrain_blocked / partially_occluded spans |
| `assignment_instability_replay` | Multiple selection blocks or divergence-class narrative events |
| `delayed_detection_replay` | First detection occurs later than sweep median (log-line index) |
| `corridor_pressure_replay` | Corridor / ingress pressure archetype (pack id heuristic) |
| `saturation_driven_ambiguity` | High ambiguity window count on saturation-family packs |
| `topology_sensitive_divergence` | Non-baseline topology with sensitivity grid contribution or pack delta |

## Deterministic classification rules

Implemented in `classify_replay_pattern.py` using bundle fields only:

1. **LOS fragmented** — `los_degraded_count` ≥ 3 OR ≥ 2 terrain_blocked/partially_occluded `los_segments`.
2. **Assignment instability** — ≥ 2 selection-category narrative events OR any divergence-category event.
3. **Delayed detection** — `first_detection_t` > sweep median + 2 (when median defined).
4. **Corridor pressure** — `pack_id` contains `corridor` OR scenario tags include `corridor`.
5. **Saturation ambiguity** — `pack_id` contains `saturation` AND `ambiguity_window_count` ≥ 2.
6. **Topology sensitive** — `pack_id` ≠ `baseline_topology_key` AND (sensitivity layer contribution OR topology_key in experiment variant set).

Members may carry multiple tags. **Primary tag** for cohort grouping = first tag in stable priority order above.

## Viewer display labels

Abbreviated labels in `platform/sa-r0-viewer` (`TAG_LABELS` in `EventPatternGroupList.tsx`) are display-only strings. Canonical identifiers remain `pattern_id` values in `PATTERN_PRIORITY` from `classify_replay_pattern.py`. CI asserts every priority pattern has a viewer label via `test_sa_platform_integrity.py`.

## Caveats

- Tags describe **replay-side concentration** in packed bundles, not live tracker health.
- Dormant lifecycle counters do not invalidate or confirm a pattern tag.
- Tags must not be displayed as readiness, severity, or effectiveness scores.
- Misclassification edge cases are possible on sparse synthetic fixtures; taxonomy is for reviewer orientation only.

## Cohort grouping

`build_replay_cohorts()` groups members sharing the same primary `pattern_id`. Singleton members form single-member cohorts. Outliers on `first_detection_t` or `los_degraded_count` (IQR-style deterministic rule) populate `anomaly_member_indices`.
