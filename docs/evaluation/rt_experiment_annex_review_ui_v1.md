# RT Experiment Annex Review UI (`rt_experiment_annex_review_ui_v1`)

**Phase:** PLAN-RT-F3 — UI planning (docs); PLAT-RT-F3 implements panels  
**Prerequisite:** PLAT-RT-F1 analytics, PLAT-RT-X1 workbench, PLAT-RT-TAC5 annex  
**Authority:** [rt_f3_experiment_annex_review_plan.md](../platform/rt_f3_experiment_annex_review_plan.md)

Read-only cognition for full tactical capture annex timelines in the RT experiment workbench.

---

## 1. Governance

| Constant | Value |
|----------|-------|
| `BANNER_ANNEX_REVIEW` | `RT ANNEX REVIEW — replay-boundary timelines only; not operational authority` |

Additive only — do not modify frozen T1/V2/X1/F1 banner strings.

Display on: annex review panel, continuity hub, annex compare strip when visible.

---

## 2. Input: `rt_tactical_capture_annex_v1`

Validated per [rt_tac1_tactical_capture_continuity_v1.md](rt_tac1_tactical_capture_continuity_v1.md):

| Rule | Detail |
|------|--------|
| `schema` | Must be `rt_tactical_capture_annex_v1` |
| `authority_label` | Must be `replay_boundary_scoped` |
| Source | Import JSON, annex bundle, or demo fixture — not live bridge pull |

Manifest `tactical_annex_summary` remains **counts-only**; full annex is optional side cache.

---

## 3. Surfaces

### 3.1 Final snapshot block

| Field | Display |
|-------|---------|
| `final_tactical_mode` | Text |
| `selected_id` | Monospace |
| `assigned_target` | Monospace |
| `authority_label` | Chip via `formatAuthorityChip` |
| `ephemeral_session_ref` | Explanatory ref — not lineage parent |
| `capture_candidate_id` | When present on annex |

### 3.2 Timeline tables (read-only)

UTC-ordered tables; label: *not synchronized to log-line clock t*.

| Section | Annex field |
|---------|-------------|
| Mode switches | `mode_switches` |
| Selected timeline | `selected_timeline` |
| Assignment timeline | `assignment_timeline` |
| TTI timeline | `tti_timeline` |
| Recommendation timeline | `recommendation_timeline` |
| Pause / resume | `pause_resume_transitions` |
| Assignment lock | `assignment_lock_events` |
| Target switch | `target_switch_events` |

Empty lists show `(empty)` — not hidden.

### 3.3 Import actions

| Action | Behavior |
|--------|----------|
| Paste annex JSON | Validate + store in `rt_experiment_annex_cache_v1` keyed by `run_id` |
| Import annex bundle | `rt_experiment_annex_bundle_v1` array |
| Load demo fixture | Vitest/dev only pattern |

---

## 4. Forbidden UI

- Winner column or readiness scores
- Operational engage/intercept language
- Live `capture_session` from browser
- SA import triggers

---

## Related

- [rt_experiment_continuity_review_v1.md](rt_experiment_continuity_review_v1.md)
- [rt_experiment_analytics_ui_v1.md](rt_experiment_analytics_ui_v1.md)
