# SA F2A — Multi-Corpus Federation Foundations Freeze Audit (PLAT-SA-F2A)

Wave plan: [sa_f2a_multi_corpus_federation_plan.md](sa_f2a_multi_corpus_federation_plan.md)

Schema docs: [replay_federation_manifest_v1.md](replay_federation_manifest_v1.md), [replay_federation_index_v1.md](replay_federation_index_v1.md), [replay_federation_lineage_v1.md](replay_federation_lineage_v1.md), [replay_federation_provenance_rules_v1.md](replay_federation_provenance_rules_v1.md), [replay_federation_recovery_continuity_v1.md](replay_federation_recovery_continuity_v1.md)

## Deliverables

| Item | Status |
|------|--------|
| F2A.1 Federation manifest + schema docs | Done |
| F2A.2 `replay_federation_lineage.py`, federation index/lineage/continuity fixtures | Done |
| F2A.3 `audit_replay_federation_integrity.py` + integrity report | Done |
| F2A.4 Read-only federation viewer panels (corpus segment) | Done |
| F2A.5 Reproducibility + snapshot builders/verifiers | Done |
| F2A.6 `audit_federation_recovery_continuity.py` + I3 `corpus_group_id` extensions | Done |
| F2A.7 Freeze audit + governance review | Done |

## Governance checks

| Check | Result |
|-------|--------|
| No WebSocket / live ROS | Pass |
| No cloud sync / collaborative editing | Pass |
| No browser orchestration or recovery authority | Pass |
| No parser/topic changes | Pass |
| Federation lineage distinct from semantic linkage | Pass |
| I2/I3 promote/quarantine invariants preserved | Pass |
| Additive-only corpus index (no F1 builder breakage) | Pass |

## Validation

```bash
python3 scripts/evaluation/build_replay_federation_index.py --check
python3 scripts/evaluation/validate_replay_federation.py
python3 scripts/evaluation/audit_replay_federation_integrity.py --check --strict
python3 scripts/evaluation/verify_replay_federation_reproducibility.py
python3 scripts/evaluation/audit_federation_recovery_continuity.py --check --strict
python3 -m pytest src/counter_uas/test/test_replay_federation_index.py \
  src/counter_uas/test/test_replay_federation_integrity.py \
  src/counter_uas/test/test_replay_federation_recovery_continuity.py -q
python3 scripts/evaluation/governance_lint_sa.py
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
scripts/ci_eval.sh tier0-sa-r0
```

## Federation / research operations summary

- **Manifest:** `fixtures/sa_r0/federation/replay_federation_manifest_v1.json` — `sa_replay_federation_r0_v1` with `canonical_r1` + `release_r1_snapshot` corpus groups.
- **Index:** `replay_federation_index_v1.json` — federated rollups and `federation_revision` fingerprint.
- **Lineage graph:** cross-group `federation_release_derived` and recovery continuity edges.
- **Publication collection:** groups F1d publication packet, research bundle, release manifest.
- **Integrity report:** orphan/stale/duplication/continuity findings under `federation/audits/`.

## Reproducibility guarantees

- Replay artifacts remain immutable; federation indexes and audits only.
- `replay_federation_reproducibility_v1` fingerprints member index revisions + publication chain.
- `replay_federation_snapshot_v1` frozen generation snapshot with file SHA256 inventory.
- Release snapshot duplication across groups documented with `supersession_note` in integrity audit.

## Limitations

- Two corpus groups over one physical corpus + release snapshot (MVP); third corpora require new registry wave.
- Cross-group byte duplication warnings expected for release-derived entries.
- Federation does not replace per-corpus F1b drift or I3 per-manifest recovery authority.

## Post-freeze stop line

**Requires new scoped wave:** distributed/cloud federation, live synchronization, collaborative editing, browser-triggered orchestration, parser changes, third+ corpora without registry discipline.

## Verdict

**Verdict: frozen** for PLAN-SA-F2A / PLAT-SA-F2A.
