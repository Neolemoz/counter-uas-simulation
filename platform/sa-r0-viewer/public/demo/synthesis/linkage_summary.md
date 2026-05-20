# Replay linkage summary

Replay linkage index for reviewer exploration only — rule-based relationships between sweep families, not causal inference.

## Nodes

| sweep_id | baseline | dominant patterns |
|----------|----------|-------------------|
| valley_sensor_sweep | valley_ingress | los_fragmented_replay |
| ridge_overlap_sweep | ridge_defense | los_fragmented_replay |
| delayed_detection_sweep | delayed_detection | los_fragmented_replay |
| saturation_assignment_sweep | saturation_ingress | los_fragmented_replay |

## Edges

- **metric_similarity** `delayed_detection_sweep` ↔ `ridge_overlap_sweep`: `delayed_detection_sweep` and `ridge_overlap_sweep` show similar ambiguity window count medians across replay variants — replay-local concentration only.
- **shared_pattern** `delayed_detection_sweep` ↔ `ridge_overlap_sweep`: `delayed_detection_sweep` and `ridge_overlap_sweep` share replay pattern `los_fragmented_replay` in reviewer exploration — explanatory similarity only.
- **storyline_reference** `delayed_detection_sweep` ↔ `ridge_overlap_sweep`: Curated storyboard references relate `delayed_detection_sweep` and `ridge_overlap_sweep` — both sweeps appear in topology-divergence showcase decks
- **metric_similarity** `delayed_detection_sweep` ↔ `saturation_assignment_sweep`: `delayed_detection_sweep` and `saturation_assignment_sweep` show similar ambiguity window count medians across replay variants — replay-local concentration only.
- **shared_pattern** `delayed_detection_sweep` ↔ `saturation_assignment_sweep`: `delayed_detection_sweep` and `saturation_assignment_sweep` share replay pattern `los_fragmented_replay` in reviewer exploration — explanatory similarity only.
- **storyline_reference** `delayed_detection_sweep` ↔ `saturation_assignment_sweep`: Curated storyboard references relate `delayed_detection_sweep` and `saturation_assignment_sweep` — ingress-timing variation themes in assignment instability walkthrough
- **metric_similarity** `delayed_detection_sweep` ↔ `valley_sensor_sweep`: `delayed_detection_sweep` and `valley_sensor_sweep` show similar ambiguity window count medians across replay variants — replay-local concentration only.
- **shared_pattern** `delayed_detection_sweep` ↔ `valley_sensor_sweep`: `delayed_detection_sweep` and `valley_sensor_sweep` share replay pattern `los_fragmented_replay` in reviewer exploration — explanatory similarity only.
- **metric_similarity** `ridge_overlap_sweep` ↔ `saturation_assignment_sweep`: `ridge_overlap_sweep` and `saturation_assignment_sweep` show similar ambiguity window count medians across replay variants — replay-local concentration only.
- **shared_pattern** `ridge_overlap_sweep` ↔ `saturation_assignment_sweep`: `ridge_overlap_sweep` and `saturation_assignment_sweep` share replay pattern `los_fragmented_replay` in reviewer exploration — explanatory similarity only.
- **shared_topology** `ridge_overlap_sweep` ↔ `saturation_assignment_sweep`: `ridge_overlap_sweep` and `saturation_assignment_sweep` share topology key `corridor_defense` in fixture replay families — not operational equivalence.
- **metric_similarity** `ridge_overlap_sweep` ↔ `valley_sensor_sweep`: `ridge_overlap_sweep` and `valley_sensor_sweep` show similar ambiguity window count medians across replay variants — replay-local concentration only.
- **shared_pattern** `ridge_overlap_sweep` ↔ `valley_sensor_sweep`: `ridge_overlap_sweep` and `valley_sensor_sweep` share replay pattern `los_fragmented_replay` in reviewer exploration — explanatory similarity only.
- **shared_topology** `ridge_overlap_sweep` ↔ `valley_sensor_sweep`: `ridge_overlap_sweep` and `valley_sensor_sweep` share topology key `valley_ingress` in fixture replay families — not operational equivalence.
- **storyline_reference** `ridge_overlap_sweep` ↔ `valley_sensor_sweep`: Curated storyboard references relate `ridge_overlap_sweep` and `valley_sensor_sweep` — valley and ridge fixture families share LOS-fragmented replay exploration themes
- **metric_similarity** `saturation_assignment_sweep` ↔ `valley_sensor_sweep`: `saturation_assignment_sweep` and `valley_sensor_sweep` show similar ambiguity window count medians across replay variants — replay-local concentration only.
- **shared_pattern** `saturation_assignment_sweep` ↔ `valley_sensor_sweep`: `saturation_assignment_sweep` and `valley_sensor_sweep` share replay pattern `los_fragmented_replay` in reviewer exploration — explanatory similarity only.
