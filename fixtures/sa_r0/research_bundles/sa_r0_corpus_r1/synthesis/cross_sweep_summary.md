# Cross-sweep replay synthesis summary

Cross-sweep replay synthesis for mentor review and research documentation only — explanatory replay rollups, not operational planning or validated doctrine.

Sweeps included: `valley_sensor_sweep, ridge_overlap_sweep, delayed_detection_sweep, saturation_assignment_sweep`

## Pattern frequency rollup

- **los_fragmented_replay**: 17 member(s) across [delayed_detection_sweep, ridge_overlap_sweep, saturation_assignment_sweep, valley_sensor_sweep]

## Ambiguity concentration comparison

- Shared hotspot cells across sweeps: `[173, 207, 212, 213]`

## Topology sensitivity rollup

- Shared topology key: `corridor_defense`
- Shared topology key: `valley_ingress`

## LOS instability rollup

- `delayed_detection_sweep`: max LOS layer count 8
- `ridge_overlap_sweep`: max LOS layer count 7
- `saturation_assignment_sweep`: max LOS layer count 6
- `valley_sensor_sweep`: max LOS layer count 5

## Divergence rollup

- Sweeps with topology-sensitive divergence: `['valley_sensor_sweep', 'ridge_overlap_sweep', 'saturation_assignment_sweep']`
- Member divergence count: 9

## Interpretation caveats

- Cross-sweep synthesis is replay-local and explanatory only.
- Shared spatial cells indicate concentration overlap in reviewer exploration — not causal proof.
- Pattern frequency counts reflect taxonomy tags on fixture members, not operational doctrine.
