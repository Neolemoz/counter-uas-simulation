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
- remaining realism frontier is sustained threshold-crossing through the existing
  `/fused_detections -> tracking_node -> /tracks/state` flow
- future work must remain additive-only, default-off, seed-controlled, parser-safe,
  and outside tracker/fusion redesign

Primary planning doc:

- [runtime_realism_plan.md](runtime_realism_plan.md)
