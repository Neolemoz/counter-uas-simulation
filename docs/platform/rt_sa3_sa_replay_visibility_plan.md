# RT-SA3 — SA Replay Tactical Visibility (PLAT-RT-SA3)

**Phase:** PLAT-RT-SA3 — read-only SA replay consumption of RT tactical annex  
**Prerequisite:** PLAT-RT-TAC5 frozen  
**Authority:** [rt_sa_replay_tactical_visibility_v1.md](../evaluation/rt_sa_replay_tactical_visibility_v1.md); [AGENTS.md](../../AGENTS.md)

**Companion artifacts:**

- [rt_sa3_governance_review_r1.md](../evaluation/rt_sa3_governance_review_r1.md)
- [rt_sa3_replay_boundary_audit.md](../evaluation/rt_sa3_replay_boundary_audit.md)
- [rt_sa3_freeze_audit.md](../evaluation/rt_sa3_freeze_audit.md)

---

## 1. Purpose

Expose frozen `rt_tactical_capture_annex_v1` inside `replay_sa_bundle_v1` and SA-R0 viewer read-only panels. No live RT hooks.

---

## 2. Architecture

RT capture staging → maintainer `replay_sa_bundle pack --rt-capture-staging` → optional `rt_tactical_replay_continuity` on bundle → SA viewer panels.

---

## 3. Allowed

| Item | Notes |
|------|-------|
| `rt_tactical_replay_continuity.py` | Pack-time embed |
| `replay_sa_bundle.py` `--rt-capture-staging` | Maintainer only |
| `sa_handoff.py` pack step wiring | When annex present |
| `platform/sa-r0-viewer/` read-only panels | No runtime controls |

## 4. Forbidden

- Live RT bridge hooks from SA viewer
- Automatic SA import / federation automation
- Parser/topic changes
- RT sandbox UI changes

---

## 5. Stop line

No RT-V2 or RT-X1 without explicit new wave audit.
