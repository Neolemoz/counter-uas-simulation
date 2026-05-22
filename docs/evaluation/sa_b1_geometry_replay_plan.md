# SA B1 — Geometry-Aware Replay Realism (PLAT-SA-B1)

**Status:** implementation wave (additive to PLAT-SA-R0)

## Goal

Improve replay-side spatial realism and topology reasoning without runtime complexity or operational semantics. Terrain and ridge topology should visibly affect replay interpretation.

## Allowed

- Additive `replay_sa_bundle_v1` fields: `los_segments`, `scenario.terrain_model`, overlay `active_t_range`, `ridge_outline_enu_m`
- `replay_sa_geometry.py` — deterministic 2.5D LOS approximation from fixture polygons
- Viewer: kind-differentiated overlays, LOS polylines, fictional heightmap (no Ion World Terrain)
- Demo fixtures under `fixtures/sa_r0/` (ridge enrichment + `demo_valley_ingress`)
- Tests, docs, `tier0-sa-r0` validation

## Forbidden

- Live ROS / rosbridge / WebSocket
- HITL, engage, readiness scoring, tactical authority
- Parser / topic / runtime changes
- Cesium Ion World Terrain as operational geography
- Changes to `GovernanceChrome` structure or authority wording

## Deliverables

| ID | Deliverable |
|----|-------------|
| B1.1 | Overlay kind styling, zone layering, `losLinks` layer toggle |
| B1.2 | Packager LOS segments + lint |
| B1.3 | Valley ingress fixture + narrative annotations |
| B1.4 | Fictional heightmap + height sampling for ridge emphasis |
| B1.5 | Freeze audit + registry entry |

## Validation

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_bundle.py src/counter_uas/test/test_replay_sa_geometry.py -q
cd platform/sa-r0-viewer && npm test && npm run build
scripts/ci_eval.sh tier0-sa-r0
```

## Post-wave

Complete [sa_b1_geometry_freeze_audit.md](sa_b1_geometry_freeze_audit.md) and register `PLAT-SA-B1` in [freeze_registry_r1.md](freeze_registry_r1.md).
