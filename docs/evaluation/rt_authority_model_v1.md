# RT Authority Model (`rt_authority_model_v1`)

**Phase:** PLAT-RT-R1a — authority labels and governance lexicon; PLAT-RT-R2e capture lexicon; PLAN-RT-R2f handoff lexicon; PLAN-RT-TAC1 tactical lexicon  
**Authority:** [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md); [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md); [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)

Defines authority labels for RT surfaces. Labels are **additive metadata** on telemetry payloads and capture artifacts.

---

## 1. Authority labels

| Label | Applies to | Meaning |
|-------|------------|---------|
| `command_authoritative` | `EntityRegistry`, stub `entity_pose_mirror`, `world_summary.revision` | Bridge command truth for sandbox world |
| `explanatory_sync` | `PoseSyncMirror`, `sync_health` on `world_summary` | Drift/stale/mismatch evidence only |
| `explanatory_telemetry` | `TelemetryMirror`, adapter-fed channels, `world_revision_hint` | Read model; not replay authority |
| `replay_boundary_scoped` | Normalized manifests, provenance, conversion | RT staging artifact; SA import requires approval |
| `tactical_recommendation_explanatory` | Assisted-mode recommendation packets | Suggest/review only — not assignment until user approval |
| `user_approval_authoritative` | Assisted-mode post-`approve_recommendation` commits | User gate before adapter motion intent |
| `tactical_controller_authoritative` | Autonomous-mode sandbox assignment loop | RT session scoped — not SA replay or engine engage authority |

---

## 2. Telemetry payload fields

| Field | Role |
|-------|------|
| `source` | Origin surface: `bridge_registry`, `bridge_session`, `adapter_feedback`, `adapter_telemetry`, `rt_tactical_controller` (future) |
| `authority_label` | One of the labels above for the payload primary semantics |
| `governance_banner` | Required RT sandbox banner on subscription events |

When `world_summary` includes both registry revision and `sync_health`, registry fields are `command_authoritative`; sync overlay fields are `explanatory_sync` (documented; single payload uses primary label `command_authoritative`).

---

## 3. Governance lexicon

| Term | Definition |
|------|------------|
| **Runtime truth** | Bridge `EntityRegistry` poses while session is active — command-authoritative |
| **Bridge intent** | Accepted bridge commands and their effects on registry; frozen into `snapshot.json` at capture |
| **Captured runtime truth** | Immutable raw/normalized staging at `capture_utc` — not live session state after teardown |
| **Adapter runtime truth** | G3/G4 mirror snapshots in `NormalizationContext` at capture instant — explanatory |
| **Normalized replay-ready artifact** | `rt_normalized_capture_v1` with `replay_boundary_scoped` — not SA authority until external import |
| **Explanatory-only mismatch** | Pose divergence in `capture_pose_cognition` / capture audit — never overrides `command_pose` |
| **Explanatory telemetry** | Mirrors, subscription rings, capture summaries — not operational state |
| **Replay authority** | SA corpus / parser-visible summaries only — never RT session mirrors |
| **Transient runtime state** | All RT session surfaces; ephemeral; no corpus lineage |
| **Runtime capture** | Raw PLAT-RT-S5 staging at `capture_session` — not SA replay authority |
| **Normalized export** | G5 replay-ready staging under `runs/rt_sandbox/captures/` — `replay_boundary_scoped` |
| **Maintainer import** | Human-gated corpus commit after external SA packaging — starts SA lineage |
| **SA replay authority** | Parser/corpus-visible truth **only** after maintainer import into corpus paths |
| **Explanatory refs** | `ephemeral_session_ref`, RT mirrors, handoff audit entries, `rt_handoff_review_v1` |
| **Sandbox candidate** | RT entity under tactical review — not operational target designation |
| **Sandbox assignment** | RT `assigned_candidate_id` commit — not weapon release or engage authority |
| **Tactical mode** | `manual` \| `assisted` \| `autonomous` on RT session — not SA replay authority |
| **Tactical recommendation** | Assisted-mode controller output — explanatory until user approval |
| **Tactical capture annex** | `rt_tactical_capture_annex_v1` — `replay_boundary_scoped`; explanatory timelines at import |

---

## 4. Surface map

| Surface | Label |
|---------|-------|
| `EntityRegistry` | `command_authoritative` |
| `PoseSyncMirror` | `explanatory_sync` |
| `TelemetryMirror` | `explanatory_telemetry` |
| `rt_normalized_capture_v1` | `replay_boundary_scoped` |
| `rt_tactical_state_v1` (future) | Mode-dependent — see [rt_tac1_tactical_modes_v1.md](rt_tac1_tactical_modes_v1.md) |
| `rt_tactical_recommendation_v1` (future) | `tactical_recommendation_explanatory` |
| `rt_tactical_capture_annex_v1` (future) | `replay_boundary_scoped` |

---

## 5. PLAN-RT-TAC1 tactical authority (docs only)

No runtime emission in PLAN-RT-TAC1. PLAT-RT-TAC2+ implement per [rt_tac1_tactical_controller_architecture_plan.md](../platform/rt_tac1_tactical_controller_architecture_plan.md).

| Mode | `assigned_candidate_id` | `tti_s` / `tactical_health` |
|------|-------------------------|----------------------------|
| Manual | `command_authoritative` (user via bridge) | `explanatory_telemetry` |
| Assisted | `user_approval_authoritative` after approve | `explanatory_telemetry` |
| Autonomous | `tactical_controller_authoritative` | `explanatory_telemetry` |

Engine `[TACTICAL_*]` logs and `/interceptor/selected_id` remain **explanatory** or engine-scoped — never SA corpus authority. See [rt_tac1_tactical_logic_reuse_v1.md](rt_tac1_tactical_logic_reuse_v1.md).

---

## Related

- [rt_revision_vocabulary_v1.md](rt_revision_vocabulary_v1.md)
- [rt_adapter_telemetry_v1.md](rt_adapter_telemetry_v1.md)
- [rt_capture_normalization_v1.md](rt_capture_normalization_v1.md)
- [rt_capture_pose_cognition_v1.md](rt_capture_pose_cognition_v1.md)
- [rt_runtime_export_semantics_v1.md](rt_runtime_export_semantics_v1.md)
- [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)
- [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md)
- [rt_sa_lineage_protection_v1.md](rt_sa_lineage_protection_v1.md)
- [rt_tac1_tactical_modes_v1.md](rt_tac1_tactical_modes_v1.md)
- [rt_tac1_tactical_governance_v1.md](rt_tac1_tactical_governance_v1.md)
