# RT-F4 — Realism Review R1 (PLAN-RT-F4)

**Contract:** [rt_runtime_realism_expansion_v1.md](rt_runtime_realism_expansion_v1.md)

## Misread risk assessment

| Feature | Risk | Mitigation |
|---------|------|------------|
| Contour overlays | Medium — may read as survey GIS | Default **off**; label “explanatory contour — not survey data” |
| Elevation bands | Low–Med | Tied to fictional heightmap; ridge labels unchanged |
| Vegetation markers | Low | Distinct styling; “fictional vegetation cue” |
| LOS / visibility hints | Medium | “Heuristic” in UI; no “detected” lexicon |
| Sensor dome context | Medium | Retain “nominal — not coverage proof” |

## Comparison to V2

V2 established fictional terrain and occlusion heuristics. F4 fills documented gaps (contours, bands, richer markers) without changing authority semantics.

**Recommendation:** Approve PLAT implementation with conservative defaults and additive banners.
