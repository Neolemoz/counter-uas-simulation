# Replay analytics summary — `ridge_overlap_sweep`

Deterministic replay sweep for spatial analytics exploration only — explanatory replay family, not operational planning.

## Dominant replay patterns (explanatory)

- Detection timing variability increases across replay variants in this sweep family.
- Most replay variants show LOS degradation concentration near ridge or masking overlays.

## Outcome histogram (replay-local)

- **ambiguity_window_count**: `[2, 2, 2, 2]`
- **first_detection_t**: `[3, 9, 3, 5]`
- **los_degraded_count**: `[11, 9, 4, 12]`

## Spatial layers

Precomputed concentration grids in `sweep.json` → `spatial_aggregate.layers`.
Explanatory replay concentration only — not operational prediction.

