# Replay Corpus Drift Report (`replay_corpus_drift_report_v1`)

Deterministic maintainer-facing drift inventory for the replay corpus. **Not** operational status or deployment readiness.

See also: [replay_corpus_index_v1.md](replay_corpus_index_v1.md), [replay_corpus_provenance_rules_v1.md](replay_corpus_provenance_rules_v1.md).

## Artifact placement

| Location | Notes |
|----------|-------|
| `fixtures/sa_r0/corpus_audits/replay_corpus_drift_report_v1.json` | Canonical drift report |
| `fixtures/sa_r0/corpus_audits/corpus_drift_summary.md` | Human-readable summary |

## Root fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_corpus_drift_report_v1` |
| `schema_version` | yes | `replay_corpus_drift_report_v1` |
| `corpus_id` | yes | Parent corpus |
| `index_revision` | yes | From corpus index at report time |
| `generation_revision` | yes | `f1b_v1` (bumped when drift logic changes) |
| `governance` | yes | Notice + anti-claims |
| `summary` | yes | Counts by `kind` and `severity` |
| `findings` | yes | Sorted drift findings |

## Finding fields

| Field | Required | Notes |
|-------|----------|-------|
| `finding_id` | yes | Stable slug `{kind}__{seq}` |
| `kind` | yes | See below |
| `severity` | yes | `info`, `warning`, `error` |
| `message` | yes | Explanatory text |
| `entry_id` | no | Related index entry |
| `path` | no | Repo-relative path |
| `evidence` | no | Structured context |

### `kind` values

| Kind | Meaning |
|------|---------|
| `stale_sha256` | Indexed artifact bytes changed since index build |
| `missing_artifact` | `primary_artifact_path` not on disk |
| `index_stale` | Committed index differs from rebuild |
| `orphan_entry` | Entry has unexpected empty lineage (non-root) |
| `unindexed_file` | Discoverable artifact not referenced in index |
| `corpus_ref_missing` | Artifact should have `corpus_ref` but does not |
| `corpus_ref_mismatch` | `corpus_ref` stale vs current index |
| `viewer_mirror_stale` | Viewer demo mirror differs from canonical index |

## Governance

**Do:** Use for maintainer regen decisions; treat as explanatory integrity only.

**Don't:** Imply tactical failure, readiness downgrade, or authority revocation.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `build_replay_corpus_drift_report.py` |
| Consumer | `audit_replay_lineage.py`, `audit_sa_platform_integrity.py` |
