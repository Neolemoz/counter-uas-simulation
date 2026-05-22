# SA Platform Release Checkpoint

**Branch:** `codex/replay-narrative-tooling-r1`  
**Checkpoint scope:** PLAT-SA-F1a through PLAT-SA-F1d (corpus operations stack) on frozen platform through PLAT-SA-STAB + E2  
**Status:** merge-ready F1 corpus checkpoint — documentation, fixtures, viewer, and integrity gates only; no new platform wave.

`AGENTS.md` remains primary governance authority.

## F1 stack freeze inventory (PLAT-SA-F1a–F1d)

| ID | Wave | Audit | Primary artifacts |
|----|------|-------|-------------------|
| PLAT-SA-F1a | Corpus indexing & lineage | [sa_f1a_corpus_indexing_freeze_audit.md](sa_f1a_corpus_indexing_freeze_audit.md) | `replay_corpus_index_v1.json`, `build_replay_corpus_index.py`, `replay_corpus_lineage.py`, release snapshot `sa_r0_corpus_r1_r1` |
| PLAT-SA-F1b | Corpus audit & provenance | [sa_f1b_corpus_audit_operations_freeze_audit.md](sa_f1b_corpus_audit_operations_freeze_audit.md) | drift report, release diff, `run_replay_corpus_regen.py`, `verify_replay_corpus_reproducibility.py` |
| PLAT-SA-F1c | Corpus navigation & reviewer UX | [sa_f1c_corpus_navigation_freeze_audit.md](sa_f1c_corpus_navigation_freeze_audit.md) | corpus browser, lineage nav, drift surfacing, `?corpus_entry=` / navigation metadata |
| PLAT-SA-F1d | Long-horizon synthesis & publication | [sa_f1d_long_horizon_publication_freeze_audit.md](sa_f1d_long_horizon_publication_freeze_audit.md) | evolution manifest/summary, publication packet, release archive export, chronology panel |

Full platform index (R0–STAB, E1/E2): [freeze_registry_r1.md](freeze_registry_r1.md).

## Corpus infrastructure maturity

The F1 stack completes deterministic **offline corpus operations** on top of the frozen replay experimentation platform:

1. **Structural index** — 57-entry `replay_corpus_index_v1` with lineage DAG, navigation, and evolution metadata
2. **Audit & reproducibility** — drift inventory, release diff, provenance audit, regen orchestration
3. **Reviewer navigation** — corpus browser, filters, chronology grouping, deep links, workstation drift badges
4. **Long-horizon cognition** — evolution manifest/summary, multi-release browser, publication packet, zip archive export
5. **Three-tier artifacts** — source fixtures, viewer mirrors, research-bundle offline archive

Regeneration: [sa_platform_maintainer_checklist.md](sa_platform_maintainer_checklist.md), [replay_corpus_regen_workflow_v1.md](replay_corpus_regen_workflow_v1.md). Reviewer entry: [sa_r0_reviewer_quickstart.md](sa_r0_reviewer_quickstart.md).

## Lineage / provenance / release capabilities

| Capability | Mechanism |
|------------|-----------|
| Entry discovery | `build_replay_corpus_index.py` |
| Lineage validation | `replay_corpus_lineage.py`, `audit_replay_lineage.py` |
| Drift surfacing | `build_replay_corpus_drift_report.py` + viewer provenance panel |
| Release snapshot | `build_replay_corpus_release.py` → `fixtures/sa_r0/corpus_releases/sa_r0_corpus_r1_r1/` |
| Cross-release diff | `diff_replay_corpus_releases.py` |
| Evolution chronology | `build_replay_corpus_evolution.py` |
| Publication inventory | `build_replay_corpus_publication.py` |
| Offline archive | `export_replay_corpus_release.py` → `corpus_releases/<id>/archive/` |
| F1 regen | `gen_f1_corpus_fixtures.py`, `gen_f1d_publication_fixtures.py` |

## Governance boundaries preserved

- Replay-only, explanatory semantics; mirrors ≠ authority
- No parser/topic/schema changes on this checkpoint
- No HITL, readiness, operational, or WebSocket expansion
- No tracker/fusion/runtime redesign
- Viewer does not execute regen or external publish
- Additive-only evolution; freeze-before-expansion discipline maintained

## Validation results (F1 release gate)

Run from repository root:

```bash
python3 -m pytest src/counter_uas/test/ -q --tb=short
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
scripts/ci_eval.sh tier0-sa-r0
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
python3 scripts/evaluation/build_replay_corpus_evolution.py --check
python3 scripts/evaluation/build_replay_corpus_publication.py --check
python3 scripts/evaluation/verify_replay_corpus_release.py
```

**Checkpoint run (F1 gate):** all green — 221 pytest; 44 viewer tests; `tier0-sa-r0` OK; 22 integrity checks OK; F1 evolution/publication/release verify OK; narrative-duplicate warnings only.

## Remaining intentional limitations

- **Info-level drift:** optional `corpus_ref_missing` on sweep manifests; meta-corpus JSON (`evolution_*`, `publication_packet`) listed as `unindexed_file` until optional index backfill
- **Single frozen release:** `sa_r0_corpus_r1_r1` only; `r2` optional via `--parent-release` when index revision changes
- **Evolution narratives:** rule-based rollups, not causal inference
- **Publication archive:** offline fixture zip, not CDN distribution
- Duplicate narrative bullets / pattern-tag drift: integrity **warnings only**
- PNG/binary byte parity: not enforced on every CI run
- Runtime research frontier: separate from platform; not started on this checkpoint

## Recommended direction after F1

**G1 complete:** [sa_platform_governance_review_r1.md](sa_platform_governance_review_r1.md), [sa_platform_maturity_assessment_r1.md](sa_platform_maturity_assessment_r1.md), [sa_platform_frontier_review_r1.md](sa_platform_frontier_review_r1.md). **Posture:** stabilize and maintain (P1 hygiene); defer new platform features until an explicit scoped wave is approved.

Do **not** start a new platform frontier until explicit governance review. Candidate post-F1 themes (planning only; see frontier review for priorities):

- Multi-corpus IDs and automated release promotion workflow
- Richer cross-release HTML publication packets (still static)
- Optional `corpus_ref` backfill on sweep manifests to clear info drift findings
- PLAN-VIZ-R2 / runtime realism: remain separate frontiers per `AGENTS.md`

## Merge readiness

- Commit F1 stack artifacts + docs + viewer + CI gates on `codex/replay-narrative-tooling-r1`
- Push branch; update PR description with this checkpoint summary before merge to `main`
- Post-merge: no new platform frontier until explicit governance review
