# RT↔SA Lineage Protection (`rt_sa_lineage_protection_v1`)

**Phase:** PLAN-RT-R2f — lineage rules for RT-origin captures  
**Authority:** [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md); [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)

Normative rules preventing RT transient IDs from becoming SA corpus or federation lineage authorities.

---

## 1. Core rules

1. **`session_id` is never lineage authority** — may appear as `ephemeral_session_ref` or explanatory cross-ref only.
2. **Conversion refs stay non-authoritative until import** — `runtime_to_replay_conversion_v1` declares pipeline inputs; it does not create corpus `parent_ref`.
3. **SA lineage starts only after explicit maintainer import** — post-packaging `run_id`, `bundle_path`, `corpus_ref` enter [replay_corpus_lineage_v1.md](replay_corpus_lineage_v1.md) semantics.
4. **Federation never auto-updates from RT** — no RT write path to `replay_federation_*` ([rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md) §3).

---

## 2. ID classification

| ID / field | Staging | Lineage `parent_ref`? | Notes |
|------------|---------|----------------------|-------|
| `session_id` | Yes | **Never** | Prefer `ephemeral_session_ref` in new artifacts |
| `ephemeral_session_ref` | Yes | **Never** | Explanatory cross-ref to RT session audit |
| `capture_candidate_id` | Yes | RT staging key only | Stable key under `runs/rt_sandbox/captures/<id>/` |
| `conversion_revision` | Normalized + conversion | RT normalization revision only | Not corpus generation |
| `run_id` / `bundle_path` / `corpus_ref` | SA bundle after pack | **Yes** — after maintainer import | SA authority |

---

## 3. Manifest lint alignment

Implementation enforces boundary rules ([export_boundary.py](../../platform/rt-sandbox-bridge/rt_sandbox/export_boundary.py)):

| Check | Function |
|-------|----------|
| Normalized manifest | `validate_normalized_manifest` — rejects `session_id` as `parent_ref`, forbids `authoritative_parent_ref` |
| Conversion manifest | `validate_conversion_manifest` — rejects `session_id` / `parent_session_id` as authoritative parent |

Tests: `test_conversion_manifest_rejects_session_id_parent`, `test_normalized_manifest_no_session_lineage` in `test_rt_sandbox_bridge.py`.

**R2f does not change lint behavior** — documents existing protection.

---

## 4. Conversion manifest rules

`runtime_to_replay_conversion_v1` must:

- Include `origin: rt_sandbox_capture_v1` (or superset containing it)
- Reference normalized staging via `staging_refs` (not external audit paths)
- List `conversion_steps` matching maintainer pipeline — bridge does not execute them
- **Not** set `parent_ref` to RT `session_id`

Failed or non-importable session states must not produce importable conversion manifests ([rt_capture_continuity_v1.md](rt_capture_continuity_v1.md) §6).

---

## 5. Federation and corpus

| Action | Allowed from RT? |
|--------|------------------|
| Write `fixtures/sa_r0/` corpus paths | **No** — blocked prefixes |
| Update federation manifest / publication collection | **No** |
| Auto-index capture in corpus browser | **No** |
| Maintainer manual import after SA pack | **Yes** — outside RT bridge |

---

## 6. Anti-patterns

| Anti-pattern | Why forbidden |
|--------------|---------------|
| Use `session_id` as `parent_ref` in SA bundle | Escalates transient RT state to replay lineage |
| Treat `capture_pose_cognition` flags as operational truth | Explanatory only (R2e) |
| Auto-run `replay_sa_bundle_pack` on `capture_session` | Violates `capture_session ≠ SA import` |
| Load RT staging JSON in SA viewer as live truth | SA viewer is static replay only |
| Merge H3 queue capture semantics into RT bridge without audit | Separate entrypoints |

---

## Related

- [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md)
- [replay_federation_lineage_v1.md](replay_federation_lineage_v1.md)
- [rt_capture_normalization_v1.md](rt_capture_normalization_v1.md)
