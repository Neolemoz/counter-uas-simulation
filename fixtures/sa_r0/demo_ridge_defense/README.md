# Demo ridge defense replay bundle

Committed `index.json` for SA-R0 viewer demos. Topology source: `fixtures/scenarios/ridge_defense/`.

Regenerate:

```bash
python3 -c "
import json, sys
from pathlib import Path
sys.path.insert(0, 'scripts/evaluation')
import replay_observability as obs, replay_static_visualization as viz, replay_sa_bundle as sa
repo = Path('.').resolve()
root = repo / 'fixtures/sa_r0/demo_ridge_defense'
pack = repo / 'fixtures/scenarios/ridge_defense'
log, meta = root/'demo.log', root/'demo.meta.json'
narrative = json.loads((repo/'src/counter_uas/test/fixtures/replay_narrative_minimal.json').read_text())
narrative['lineage']['log_path'] = str(log.relative_to(repo))
narrative['lineage']['meta_path'] = str(meta.relative_to(repo))
single = obs.build_single_run_report(log, meta_path=meta)
manifest = viz.build_visualization_manifest(narrative, observability=single)
bundle = sa.build_replay_sa_bundle(narrative, observability=single, viz_manifest=manifest, scenario_pack_path=pack)
(repo/'fixtures/sa_r0/demo_ridge_defense/index.json').write_text(json.dumps(bundle, indent=2, sort_keys=True, default=str)+'\n')
"
cp fixtures/sa_r0/demo_ridge_defense/index.json platform/sa-r0-viewer/public/demo/index.json
```
