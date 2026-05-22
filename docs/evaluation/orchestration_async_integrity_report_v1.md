# Orchestration Async Integrity Report (`orchestration_async_integrity_report_v1`)

**Phase:** PLAT-SA-I2 — corpus async dimension audit  
**Authority:** [AGENTS.md](../../AGENTS.md)

Explanatory async integrity audit output from `audit_orchestration_async_integrity.py`. **Not** parser-visible.

---

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | `orchestration_async_integrity_report_v1` |
| `schema_version` | string | `1` |
| `checked_at` | string | ISO-8601 UTC |
| `ok` | boolean | Corpus pass when no errors |
| `governance` | object | `notice`, `anti_claims` |

## Recommended fields

| Field | Type | Description |
|-------|------|-------------|
| `strict` | boolean | Strict mode used |
| `issues` | array | `{kind, manifest_id, message}` error entries |
| `warnings` | array | Non-fatal findings |
| `per_manifest` | object | Per-manifest async audit detail |
| `governance_banner` | string | Viewer mirror banner |

---

## Audit dimensions

See [experiment_orchestration_async_governance_v1.md](experiment_orchestration_async_governance_v1.md).

I1 `audit_orchestration_integrity.py --strict` remains required separately.

*End of orchestration async integrity report v1.*
