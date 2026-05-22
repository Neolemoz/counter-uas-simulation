# Orchestration Async Batch Audit (`orchestration_async_batch_audit_v1`)

**Phase:** PLAT-SA-I3 — corpus-scale async review summary  
**Authority:** [AGENTS.md](../../AGENTS.md)

Explanatory batch summary from `audit_orchestration_recovery.py --batch`. **Not** parser-visible.

---

## Required fields

| Field | Type | Description |
|-------|------|-------------|
| `artifact_type` | string | `orchestration_async_batch_audit_v1` |
| `schema_version` | string | `1` |
| `generated_at` | string | ISO-8601 UTC |
| `ok` | boolean | Corpus recovery audit pass |
| `governance` | object | `notice`, `anti_claims` |

## Recommended fields

| Field | Type | Description |
|-------|------|-------------|
| `async_manifest_count` | number | Async sidecars scanned |
| `status_counts` | object | Counts by `async_execution_status` |
| `recovery_issue_count` | number | Recovery audit errors |
| `recovery_warning_count` | number | Recovery audit warnings |
| `failed_manifest_ids` | array | Manifests with `failed` or recovery errors |
| `quarantined_manifest_ids` | array | Manifests with `quarantined` |
| `governance_banner` | string | Viewer mirror banner |

---

## Usage

Maintainers run recovery audit in strict mode, then `--batch` to refresh `fixtures/orchestration/synthesis/async_batch_audit_v1.json`. Sync to viewer via `sync_orchestration_mirrors.py`.

*End of orchestration async batch audit v1.*
