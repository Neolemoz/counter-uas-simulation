# Experiment Orchestration Operations (v1)

**Phase:** PLAN-SA-I1 — deterministic orchestration operations layer  
**Authority:** [AGENTS.md](../../AGENTS.md); extends [experiment_job_manifest_v1.md](experiment_job_manifest_v1.md) (H3).

Maintainer-facing **operations** for corpus-wide orchestration hygiene: integrity audits, ops promotion, replay continuity, and mirror sync. CLI remains authoritative.

---

## Coarse lifecycle groups

| Group | `operations_status` values | Meaning |
|-------|---------------------------|---------|
| **Editable** | `pending` | Manifest/ops sidecar may change |
| **Validated** | `validated` | Manifest lint + validation mirrors current |
| **Released** | `queued`, `executed` | Queue snapshot recorded; run bookkeeping done |
| **Replay-ready** | `replay_generated` | Bundle/corpus outputs verified |
| **Retired** | `archived` | Terminal; read-only semantics |

---

## Operations commands

### Integrity audit (corpus-wide)

```bash
python3 scripts/evaluation/audit_orchestration_integrity.py
python3 scripts/evaluation/audit_orchestration_integrity.py --strict
python3 scripts/evaluation/audit_orchestration_integrity.py --json
python3 scripts/evaluation/audit_orchestration_integrity.py --lineage-report --manifest-id valley_ingress_validation_only
python3 scripts/evaluation/audit_orchestration_integrity.py --batch --pipeline validation-only
```

### Manifest lint (all orchestration manifests)

```bash
python3 scripts/evaluation/lint_experiment_manifest.py fixtures/orchestration/manifests/ --check
python3 scripts/evaluation/lint_orchestration_ops_manifest.py --all-manifests
```

### Ops promotion

```bash
python3 scripts/evaluation/promote_experiment_manifest.py \
  fixtures/orchestration/manifests/valley_ingress_validation.json \
  --init
python3 scripts/evaluation/promote_experiment_manifest.py \
  fixtures/orchestration/manifests/valley_ingress_validation.json \
  --status validated --record-validation
python3 scripts/evaluation/promote_experiment_manifest.py \
  fixtures/orchestration/manifests/ridge_defense_synthetic.json \
  --status queued --record-queue --dry-run
python3 scripts/evaluation/promote_experiment_manifest.py \
  fixtures/orchestration/manifests/ridge_defense_synthetic.json \
  --repro-check --json
python3 scripts/evaluation/promote_experiment_manifest.py \
  fixtures/orchestration/manifests/ridge_defense_synthetic.json \
  --summary --lineage-report
```

### Mirror sync

```bash
python3 scripts/evaluation/sync_orchestration_mirrors.py
```

Includes `integrity_report.json` for viewer read-only panel.

---

## Sync chain (orchestration authority)

`lint_experiment_manifest` → `promote --record-validation` → `run_experiment_queue` (dry-run or live) → `promote --record-queue` → `promote --status replay_generated` (when outputs verified) → `sync_orchestration_mirrors` → optional `sync_authoring_mirrors` (handoff back-check).

Authoring chain remains upstream: see [scenario_authoring_operations_v1.md](scenario_authoring_operations_v1.md).

---

## Retirement

```bash
python3 scripts/evaluation/promote_experiment_manifest.py \
  fixtures/orchestration/manifests/<manifest>.json --status archived
```

*End of experiment orchestration operations v1.*
