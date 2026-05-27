# RT Tactical Capture Continuity (`rt_tac1_tactical_capture_continuity_v1`)

**Phase:** PLAT-RT-TAC5 — implemented  
**Authority:** [rt_capture_continuity_v1.md](rt_capture_continuity_v1.md); [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md); [rt_runtime_export_semantics_v1.md](rt_runtime_export_semantics_v1.md); [rt_tac5_tactical_capture_continuity_plan.md](../platform/rt_tac5_tactical_capture_continuity_plan.md)

Defines tactical timeline preservation in normalized capture for sandbox review.

---

## 1. Relationship to RT-S5 / G5 capture

Existing capture pipeline ([rt_capture_normalization_v1.md](rt_capture_normalization_v1.md)) freezes:

- Bridge command poses (`command_authoritative`)
- Adapter mirrors (`explanatory_sync`, `explanatory_telemetry`)
- Pose cognition tri-source interpretation (`explanatory-only mismatch`)

TAC5 **adds** a tactical timeline annex — still `replay_boundary_scoped`, not SA authority until maintainer import.

---

## 2. Schema: `rt_tactical_capture_annex_v1`

Embedded in `rt_normalized_capture_v1` as optional `tactical_annex`; sidecar `tactical_annex.json` under capture staging.

| Field | Description |
|-------|-------------|
| `schema` | `rt_tactical_capture_annex_v1` |
| `origin` | `rt_sandbox_tactical_v1` |
| `capture_candidate_id` | UUID |
| `ephemeral_session_ref` | Session ref (explanatory; not lineage parent) |
| `final_tactical_mode` | Mode at capture |
| `selected_id` | Final selected interceptor id |
| `assigned_target` | Final assigned target id |
| `selected_timeline` | Selection history |
| `assignment_timeline` | Assignment commits/clears |
| `tti_timeline` | Throttled TTI samples |
| `recommendation_timeline` | Assisted recommendation events |
| `mode_switches` | Manual/Assisted/Autonomous transitions |
| `pause_resume_transitions` | Autonomous pause/resume |
| `assignment_lock_events` | Autonomous lock windows |
| `target_switch_events` | Selected/assigned pair changes |
| `authority_label` | `replay_boundary_scoped` |
| `governance_banner` | Required explanatory banner |

**Bounds:** default max 256 entries per timeline list, 64 TTI samples (see `tactical_capture_buffer.py`).

---

## 3. Timeline entries

### 3.1 `selected_timeline[]`

| Field | Type | Meaning |
|-------|------|---------|
| `t_utc` | ISO-8601 | Sample time |
| `candidate_id` | string \| null | Highlighted entity |
| `role` | `interceptor` \| `target` | Selection role |
| `source` | `user` \| `controller` \| `recommendation` | Who set highlight |
| `authority_label` | string | At sample time |

### 3.2 `assignment_timeline[]`

| Field | Type | Meaning |
|-------|------|---------|
| `t_utc` | ISO-8601 | Commit time |
| `assigned_candidate_id` | string \| null | Committed interceptor |
| `previous_id` | string \| null | Prior interceptor |
| `reason` | string | e.g. `user_assign`, `approved_recommendation`, `autonomous_commit`, `clear` |
| `authority_label` | string | |

### 3.3 `tti_timeline[]`

| Field | Type | Meaning |
|-------|------|---------|
| `t_utc` | ISO-8601 | Sample time |
| `candidate_id` | string | Interceptor evaluated |
| `tti_s` | number \| null | Cap-speed TTI |
| `feasible` | boolean | |
| `reason` | string | FeasibilityDecision-compatible |

### 3.4 `recommendation_timeline[]`

| Field | Type | Meaning |
|-------|------|---------|
| `t_utc` | ISO-8601 | Event time |
| `event` | `issued` \| `approved` \| `rejected` | |
| `recommendation_id` | string \| null | |
| `detail` | object \| null | Optional snapshot |

### 3.5 `mode_switches[]` / `pause_resume_transitions[]` / `assignment_lock_events[]` / `target_switch_events[]`

See PLAT-RT-TAC5 implementation and [rt_tac5_freeze_audit.md](rt_tac5_freeze_audit.md).

---

## 4. Capture rules

| Rule | Rationale |
|------|-----------|
| Annex written at `capture_session` from `TacticalCaptureBuffer` | Point-in-time freeze |
| Empty buffer → no sidecar; `tactical_capture_annex_empty` audit | No false annex |
| Failed / `discarded` sessions **must not** capture | [rt_capture_continuity_v1.md](rt_capture_continuity_v1.md) §6 |
| `ephemeral_session_ref` not lineage parent | SA lineage protection |
| Cross-session reads **forbidden** | Multi-session isolation |
| Pose cognition still independent | Tactical annex does not override `command_pose` |

---

## 5. SA import interaction

| Stage | Tactical annex role |
|-------|---------------------|
| RT staging | `tactical_annex.json` + embedded normalized block |
| Normalization | Validated optional section |
| Maintainer import | Explanatory reviewer context only |
| SA viewer | Read-only tactical continuity panels (PLAT-RT-SA3) — [rt_sa_replay_tactical_visibility_v1.md](rt_sa_replay_tactical_visibility_v1.md) |
| Parser | **No** new parser-visible fields |

---

## 6. Audit events (capture-time)

See [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md) §6:

- `tactical_capture_annex_written` / `tactical_capture_annex_empty`
- Rollups: `tactical_capture_snapshot`, `tactical_switch`, `tactical_assignment`, `tactical_lock`, `tactical_pause_resume`

Runtime tactical audits (`tactical_mode_changed`, etc.) unchanged from TAC2–TAC4.

---

## Related

- [rt_tac5_freeze_audit.md](rt_tac5_freeze_audit.md)
- [rt_tac1_tactical_telemetry_v1.md](rt_tac1_tactical_telemetry_v1.md)
- [rt_roadmap_tac1_tac5_v1.md](rt_roadmap_tac1_tac5_v1.md)
