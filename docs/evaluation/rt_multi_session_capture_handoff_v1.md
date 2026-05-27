# RT Multi-Session Capture / Handoff (`rt_multi_session_capture_handoff_v1`)

**Phase:** PLAN-RT-M1 — multi-session architecture (docs only)  
**Authority:** [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md); [rt_capture_normalization_v1.md](rt_capture_normalization_v1.md); [rt_sa_import_bridge_v1.md](rt_sa_import_bridge_v1.md)

Per-session capture and handoff isolation for local multi-session RT sandbox. **No implementation in PLAN-RT-M1.**

---

## 1. Per-session capture

Each session captures independently:

| Rule | Value |
|------|-------|
| Command | `capture_session` with target `session_id` |
| Precondition | Target session state is `stopped` only |
| World source | Target `SessionRecord.world` only — never cross-read sibling sessions |
| Post-capture | Target session → `captured`; teardown via `teardown_capture_post_session` |
| Registry | Evict captured session after teardown |

Capture of session A while session B is `running` is permitted — B is unaffected.

---

## 2. Staging isolation

| Artifact | Path | Isolation |
|----------|------|-----------|
| Snapshot | `runs/rt_sandbox/captures/<capture_candidate_id>/snapshot.json` | Per capture; includes `ephemeral_session_ref` |
| Capture report | `.../capture_report.json` | Per capture |
| Normalized bundle | `.../normalized/` (G5) | Per capture |
| Handoff staging | `runs/rt_sandbox/sa_handoff/<capture_candidate_id>/` | Per capture |

Staging dir naming unchanged from PLAT-RT-S5. No shared mutable staging between sessions.

---

## 3. Metadata and lineage

| Field | Rule |
|-------|------|
| `session_id` in capture metadata | `ephemeral_session_ref` — **not** SA lineage authority |
| `capture_candidate_id` | Primary handoff correlation key |
| Cross-session refs | Forbidden in capture artifacts |

See [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md).

---

## 4. Normalization isolation

G5 normalization runs per capture candidate:

- Input: single session's staging dir
- Output: normalized artifacts in same capture dir
- Normalization audit scoped to target `capture_candidate_id`
- Normalization of capture A must not read capture B staging

---

## 5. Handoff separation

Maintainer handoff CLIs ([rt_sa_import_bridge_v1.md](rt_sa_import_bridge_v1.md)):

| CLI / event | Scope |
|-------------|-------|
| `rt_handoff_review.py` | Per `capture_candidate_id` |
| `rt_sa_import.py` | Per approved handoff bundle |
| `handoff_*` audit events | Keyed by `capture_candidate_id` |

No session-to-session promotion. No automatic SA import. Browser does not read staging.

---

## 6. Global staging caps

| Cap | Value | Scope |
|-----|-------|-------|
| `max_staged_captures` | 32 | Global across all sessions |
| `max_capture_bundle_bytes` | 5 MiB | Per capture |

When global cap exceeded → `RESOURCE_LIMIT_EXCEEDED` on next `capture_session` regardless of which session initiates.

---

## 7. UI capture panel (M2)

Capture/handoff workflow panel ([rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md) §6) binds to **selected session**:

- Capture readiness derived from selected session lifecycle
- Pipeline phases unchanged — maintainer CLIs only for staging truth
- Background session capture requires selecting that session's tab first

Browser **must not** invoke `capture_session` — cognition only (unchanged T4 rule).

---

## 8. Explicit non-goals

- Batch capture across sessions
- Cross-session snapshot merge
- Automatic replay ingestion
- SA viewer capture hooks
- Federation writes from any session

---

## Related

- [rt_multi_session_registry_v1.md](rt_multi_session_registry_v1.md)
- [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md)
- [rt_capture_continuity_v1.md](rt_capture_continuity_v1.md)
