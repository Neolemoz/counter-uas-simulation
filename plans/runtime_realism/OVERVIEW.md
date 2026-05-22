# Runtime Realism Planning Track

Purpose:

- improve autonomy robustness under more realistic runtime conditions
- preserve replay/evaluation compatibility
- preserve deterministic reproducibility where practical

Focus areas:

- sensor latency injection
- stale-track simulation
- dropped detections
- asynchronous timing and packet jitter
- intermittent visibility and false positives
- target maneuver randomness
- track fragmentation
- environmental uncertainty

Current frozen checkpoint:

- Wave 1 tracks-state-centered ambiguity realism is complete and stable
- Wave 2 lifecycle-propagation refinement is complete and stable
- Wave 3 threshold-sensitive lifecycle activation is complete and stable
- Wave 4 topology-aware lifecycle observability refinement is complete and stable
- Wave 5 threshold-crossing lifecycle activation refinement is complete and stable
- remaining realism frontier is bounded cadence / phase refinement inside the validated
  bringup reachability geometry through the existing `bringup.launch.py ->
  /fused_detections -> tracking_node -> /tracks/state` path
- future work must remain additive-only, default-off, seed-controlled, parser-safe,
  and outside tracker/fusion redesign

Primary planning doc:

- [runtime_realism_plan.md](runtime_realism_plan.md)
