# Scenario Authoring Operations (v1)

**Phase:** PLAN-SA-A2 — deterministic authoring operations layer  
**Authority:** [AGENTS.md](../../AGENTS.md); extends [scenario_authoring_workflow_v1.md](scenario_authoring_workflow_v1.md) (A1 ladder).

Maintainer-facing **operations** for corpus-wide authoring hygiene: integrity audits, promotion reports, lifecycle retirement, and mirror sync. CLI remains authoritative.

---

## Coarse lifecycle groups

| Group | `promotion_status` values | Meaning |
|-------|---------------------------|---------|
| **Editable** | `draft`, `linted` | Pack may change; validate before promote |
| **Validated** | `validated` | Topology lint recorded; fingerprint current |
| **Released** | `promoted`, `orchestration_ready` | Eligible for catalog policy and handoff refs |
| **Retired** | `deprecated`, `archived` | Read-only semantics; no forward promote without explicit CLI |

A1 intermediate states (`linted`, `orchestration_ready`) are retained for compatibility.

---

## Operations commands

### Integrity audit (corpus-wide)

```bash
python3 scripts/evaluation/audit_scenario_authoring_integrity.py
python3 scripts/evaluation/audit_scenario_authoring_integrity.py --strict
python3 scripts/evaluation/audit_scenario_authoring_integrity.py --json
python3 scripts/evaluation/audit_scenario_authoring_integrity.py --lineage-report --pack-id valley_ingress
```

### Manifest lint (all catalog packs)

```bash
python3 scripts/evaluation/lint_scenario_authoring_manifest.py --all-catalog-packs
python3 scripts/evaluation/lint_scenario_authoring_manifest.py --all-catalog-packs --strict
```

### Promotion with summary

```bash
python3 scripts/evaluation/promote_scenario_pack.py fixtures/scenarios/<pack_id> \
  --status promoted --notes "A2 backfill" --summary --json
python3 scripts/evaluation/promote_scenario_pack.py fixtures/scenarios/<pack_id> --repro-check
python3 scripts/evaluation/promote_scenario_pack.py fixtures/scenarios/<pack_id> --diff-since-last-promote
```

### Mirror sync

```bash
python3 scripts/evaluation/sync_authoring_mirrors.py
```

Includes `integrity_report.json` for viewer read-only panel.

### Catalog sync (advisory)

```bash
python3 scripts/evaluation/sync_sa_catalog.py
python3 scripts/evaluation/sync_sa_catalog.py --strict-promotion
```

Default: warn when manifest exists and `promotion_status < promoted`. `--strict-promotion`: exit non-zero.

---

## Retirement

```bash
python3 scripts/evaluation/promote_scenario_pack.py fixtures/scenarios/<pack_id> --status deprecated
python3 scripts/evaluation/promote_scenario_pack.py fixtures/scenarios/<pack_id> --status archived
```

`archived` is terminal for forward promotion unless maintainer uses explicit downgrade policy (not automated).

---

## Sync chain (unchanged authority)

`validate` → `record-validation` → `promote` → `sync_authoring_mirrors` → `sync_orchestration_mirrors` → optional corpus regen (F1d gate separate).

*End of scenario authoring operations v1.*
