# SA Stabilization — Platform Governance Hardening (PLAT-SA-STAB)

**Status:** maintenance wave (post PLAT-SA-E2)

## Goal

Improve maintainability, reviewer reliability, reproducibility, and export integrity without expanding platform scope.

## Allowed

- Additive audit scripts and validators under `scripts/evaluation/`
- CI extensions in `scripts/ci_eval.sh tier0-sa-r0`
- Fixture ↔ `public/demo` synchronization fixes
- Documentation cleanup and freeze audit
- Governance batch lint over committed SA fixtures

## Forbidden

- New viewer features or GovernanceChrome redesign
- Compare/sweep semantic expansion, ML clustering, readiness/operational scoring
- Parser/topic/schema changes, live ROS/WebSocket/HITL
- Matplotlib regeneration on every CI run

## Integrity checks

| Check | Command / module |
|-------|------------------|
| Full audit | `python3 scripts/evaluation/audit_sa_platform_integrity.py --all` |
| Catalog sync | `python3 scripts/evaluation/check_sa_catalog_sync.py` |
| Synthesis stale | `python3 scripts/evaluation/build_cross_sweep_synthesis.py --check` |
| Linkage stale | `python3 scripts/evaluation/build_replay_linkage.py --check` |
| Research bundle | `python3 scripts/evaluation/export_research_bundle.py --check` |
| Governance batch | `python3 scripts/evaluation/governance_lint_sa.py` |
| CI gate | `scripts/ci_eval.sh tier0-sa-r0` |

## Regeneration hints (when audit fails)

```bash
python3 scripts/evaluation/sync_sa_catalog.py
python3 scripts/evaluation/gen_d3_sweep_enrichment.py
python3 scripts/evaluation/gen_e1_presentation_fixtures.py
python3 scripts/evaluation/gen_e2_research_fixtures.py
```

## Validation

```bash
python3 -m pytest src/counter_uas/test/ -q --tb=short
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
```
