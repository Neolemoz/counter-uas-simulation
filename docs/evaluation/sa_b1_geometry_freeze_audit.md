# SA B1 Geometry Replay Freeze Audit (PLAT-SA-B1)

## Scope

- [sa_b1_geometry_replay_plan.md](sa_b1_geometry_replay_plan.md)
- `scripts/evaluation/replay_sa_geometry.py` (LOS helpers, fictional heightmap)
- `scripts/evaluation/replay_sa_bundle.py` (`los_segments`, terrain_model)
- `platform/sa-r0-viewer/` overlay/LOS layers
- `fixtures/sa_r0/demo_ridge_defense/`, `fixtures/sa_r0/demo_valley_ingress/`

No runtime, parser, topic, rosbridge, or `web/` changes.

## Governance Result

**Verdict: frozen** for PLAT-SA-B1.

| Check | Result |
|-------|--------|
| Authority creep | Pass — LOS/masking labeled explanatory; caveats on segments |
| Parser safety | Pass — additive bundle fields only |
| Runtime isolation | Pass — static bundles |
| Operational semantics | Pass — no C2/engage/readiness |
| Geospatial | Pass — fictional heightmap; no Ion World Terrain |
| Governance chrome | Pass — header component unchanged |
| SA-R0 boundary | Pass — extends PLAT-SA-R0 viewer; does not reopen runtime |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_bundle.py src/counter_uas/test/test_replay_sa_geometry.py -q
cd platform/sa-r0-viewer && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-sa-r0
```

## Known Limitations

- LOS is 2.5D polygon proxy, not radar/EOIR physics
- Masking regions are fixture-authored, not Gazebo terrain extraction
- Fictional heightmap is exaggerated replay aid, not survey data
