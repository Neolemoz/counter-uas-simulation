# RT SA Replay Tactical Visibility (`rt_sa_replay_tactical_visibility_v1`)

**Phase:** PLAT-RT-SA3 — SA replay consumption contract  
**Authority:** [rt_tac1_tactical_capture_continuity_v1.md](rt_tac1_tactical_capture_continuity_v1.md); [replay_sa_bundle_schema.md](replay_sa_bundle_schema.md)

---

## 1. Core invariant

RT authority ends at `replay_sa_bundle_pack`. SA replay bundles may carry **explanatory** tactical continuity only.

---

## 2. Bundle field: `rt_tactical_replay_continuity`

Optional top-level on `replay_sa_bundle_v1`:

| Field | Required | Notes |
|-------|----------|-------|
| `schema` | yes | `rt_tactical_replay_continuity_v1` |
| `source` | yes | `rt_sandbox_capture_v1` |
| `continuity_available` | yes | `true` when annex embedded |
| `capture_candidate_id` | yes | RT capture UUID |
| `governance_banner` | yes | Explanatory-only |
| `provenance` | yes | RT capture refs; `authority_stopped_at: replay_sa_bundle_pack` |
| `tactical_annex` | yes | Copy of `rt_tactical_capture_annex_v1` |

Sidecar `tactical_annex.json` may also exist in bundle directory (maintainer inspection).

---

## 3. SA viewer surfaces

Read-only:

- RT capture origin badge
- Final tactical mode / selected / assigned snapshot
- Timeline tables (selected, assignment, TTI, recommendation, mode/pause/lock/target switches)
- Provenance: imported from RT capture

**Not synchronized** to log-line `t` clock by default (UTC list cognition).

---

## 4. Forbidden

- Live RT session commands from SA
- Authority escalation (`command_authoritative` on replay bundle)
- Parser-visible field additions

---

## Related

- [rt_sa3_freeze_audit.md](rt_sa3_freeze_audit.md)
- [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)
