# Replay Federation Provenance Rules (`replay_federation_provenance_rules_v1`)

Deterministic integrity policies for multi-corpus federation audits.

## Finding kinds (`replay_federation_integrity_report_v1`)

| Kind | Severity | Rule |
|------|----------|------|
| `orphan_corpus_group` | error | Manifest references index path that does not exist |
| `stale_federation_ref` | error | `parent_federation_ref` or lineage ref target missing |
| `replay_duplication_across_groups` | warning | Same `primary_artifact_path` + `sha256` in multiple groups without supersession note |
| `lineage_continuity_break` | error | Federation edge endpoint not in registered groups |
| `publication_continuity_break` | error | Collection member path missing or hash mismatch |
| `corpus_ref_registry_mismatch` | warning | Downstream `corpus_ref` not matching any registered group |

## Duplication policy

Cross-group duplication is permitted when:

- Release snapshot group mirrors canonical entries (expected for `federation_release_derived` edge), or
- Integrity report documents explicit `supersession_note` on the finding.

## Orphan policy

Unregistered corpus indexes under `fixtures/sa_r0/` are **not** auto-included; federation is explicit registry only.

## Relationship to F1b drift

F1b drift operates per-corpus; federation integrity operates cross-group. Both may flag the same bytes with different kinds.

## Implementation

`audit_replay_federation_integrity.py` — `--strict` elevates warnings to errors except `replay_duplication_across_groups` with release-derived note.
