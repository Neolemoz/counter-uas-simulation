# SA H3 — Offline Experiment Orchestration Freeze Audit (PLAT-SA-H3)

## Scope

- [h3_offline_experiment_orchestration_plan.md](../platform/h3_offline_experiment_orchestration_plan.md)
- [experiment_job_manifest_v1.md](experiment_job_manifest_v1.md), [experiment_run_queue_v1.md](experiment_run_queue_v1.md)
- `scripts/evaluation/experiment_orchestration.py`, `run_experiment_queue.py`, `lint_experiment_manifest.py`, `sync_orchestration_mirrors.py`
- `fixtures/orchestration/`
- `platform/sa-r0-viewer/src/orchestration/` — read-only status panels
- Corpus segment: `CorpusProvenancePanel` mount

No parser/topic changes. No viewer launch of Gazebo or regen subprocesses.

## Governance Result

**Verdict: frozen** for PLAT-SA-H3.

## Boundary Checks

| Check | Result |
|-------|--------|
| Viewer invokes sim/regen | Pass — static fetch only |
| Parser safety | Pass — no parser contract changes |
| Live ROS / WebSocket | Pass — not introduced |
| Orchestration execution from browser | Pass — CLI only |
| runtime_capture gating | Pass — requires `--allow-runtime-capture` |
| HITL / C2 / scoring | Pass — not introduced |
| Governance lint | Pass — manifests + mirrors |

## Deliverables

| Deliverable | Status |
|-------------|--------|
| Job manifest schema + fixtures | Done |
| Queue runner + reports | Done |
| Validation mirrors | Done |
| Viewer orchestration panels | Done |
| sync_orchestration_mirrors | Done |
| pytest + vitest | Done |
| tier0-sa-r0 H3 checks | Done |

## Validation

```bash
python3 scripts/evaluation/lint_experiment_manifest.py fixtures/orchestration/manifests/ --check
python3 scripts/evaluation/run_experiment_queue.py --manifest fixtures/orchestration/manifests/ridge_defense_synthetic.json --dry-run
python3 scripts/evaluation/sync_orchestration_mirrors.py
python3 -m pytest src/counter_uas/test/test_experiment_orchestration.py -q
(cd platform/sa-r0-viewer && npm test && npm run build)
python3 scripts/evaluation/governance_lint_sa.py
scripts/ci_eval.sh tier0-sa-r0
```

## Post-freeze

H4: async workers, default CI capture lane, promotion workflows — require new scoped plan.
