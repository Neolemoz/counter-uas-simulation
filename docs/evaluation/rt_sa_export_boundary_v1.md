# RT ↔ SA Export Boundary (`rt_sa_export_boundary_v1`)

**Phase:** PLAN-RT-S1 — interactive runtime sandbox (docs only)  
**Authority:** [AGENTS.md](../../AGENTS.md); frozen [experiment_workflow_scenario_to_replay_v1.md](experiment_workflow_scenario_to_replay_v1.md)

One-way boundary from **transient RT sessions** to **deterministic SA replay artifacts**. Prevents replay contamination and RT→SA authority escalation.

---

## 1. Core invariant

```
capture_session ≠ SA replay import
```

`capture_session` emits a **capture candidate** only. SA replay consumption requires explicit validation, packaging, provenance injection, governance audit, and maintainer import approval (implementation deferred to RT-S5).

---

## 2. Export pipeline

```mermaid
flowchart LR
  RTSession[RT_session_transient]
  Cap[capture_session]
  Gate1[explicit_approval_record]
  Val[validation_gate]
  Pack[replay_sa_bundle_pack]
  Prov[provenance_injection]
  Audit[governance_lint]
  Import[SA_replay_import]
  Fed[federation_index]

  RTSession --> Cap
  Cap --> Gate1
  Gate1 --> Val
  Val --> Pack
  Pack --> Prov
  Prov --> Audit
  Audit --> Import
  Import -.->|never_auto| Fed
```

| Stage | Owner | RT authority |
|-------|-------|--------------|
| `capture_session` | RT bridge | Emits candidate metadata + staging refs |
| `explicit_approval_record` | Maintainer | Human/CLI gate — not automatic |
| `validation_gate` | SA tooling (`validate_scenario.py`, etc.) | None |
| `replay_sa_bundle` pack | SA tooling (`replay_sa_bundle.py`) | None |
| `provenance_injection` | SA observability chain | None |
| `governance_lint` | `replay_observability.py governance-lint` | None |
| `SA_replay_import` | Maintainer | Commits bundle to corpus path |
| `federation_index` | SA F2A CLIs | **Never** auto-updated from RT |

---

## 3. Normative rules

1. **`capture_session` ≠ automatic SA import** — capture candidates remain in staging until maintainer initiates export steps.
2. **Capture does not imply federation visibility** — no publication collection, federation manifest, or cross-corpus index updates from RT events.
3. **RT sessions have zero corpus/federation authority** — `session_id` and bridge handles cannot be `parent_ref` in [replay_corpus_lineage_v1.md](replay_corpus_lineage_v1.md) or [replay_federation_lineage_v1.md](replay_federation_lineage_v1.md).
4. **Sequential SA import** — validation → packaging → provenance → governance audit → explicit import approval (RT-S5).
5. **Lineage authorities** — only post-packaging `run_id` / `bundle_path` / `corpus_ref` enter replay lineage; **transient runtime IDs never become lineage authorities**.
6. **Reject import** when `origin` lacks `rt_sandbox_capture_v1`, when live telemetry blobs appear without `runtime_to_replay_conversion_v1` manifest, or when `session_id` is listed as authoritative parent.

---

## 4. Capture candidate schema (stub)

**Lineage warning:** `parent_session_id` (if present) is an **explanatory cross-reference only** — it must **not** be written into [replay_corpus_lineage_v1.md](replay_corpus_lineage_v1.md) or [replay_federation_lineage_v1.md](replay_federation_lineage_v1.md) as `parent_ref` or any authoritative lineage parent. RT-S5 schema wave should prefer renaming to `ephemeral_session_ref` to avoid implementer confusion.

```json
{
  "schema": "rt_capture_candidate_v1",
  "session_id": "uuid-ephemeral",
  "parent_session_id": "uuid-ephemeral",
  "origin": "rt_sandbox_capture_v1",
  "scenario_pack_ref": "fixtures/scenarios/...",
  "capture_utc": "ISO-8601",
  "approval_status": "pending",
  "governance_banner": "CAPTURE CANDIDATE — requires validation before replay import",
  "staging_refs": {
    "log_path": null,
    "snapshot_ref": null
  }
}
```

`approval_status: pending` until `explicit_approval_record` is filed.

---

## 5. Explicit approval record (stub)

```json
{
  "schema": "rt_capture_approval_v1",
  "capture_candidate_id": "uuid",
  "approved_by": "maintainer",
  "approved_utc": "ISO-8601",
  "intent": "replay_research_import",
  "governance_banner": "MAINTAINER APPROVAL — not automatic promotion"
}
```

No approval record → export pipeline **must not** proceed.

---

## 6. Separation from H3 maintainer capture

Frozen offline path ([experiment_workflow_scenario_to_replay_v1.md](experiment_workflow_scenario_to_replay_v1.md)):

- `run_experiment_queue.py --allow-runtime-capture` → `run_capture.py`
- CLI-driven; not browser-driven

RT interactive capture is a **separate entrypoint** (RT-S5). RT-S1 documents boundaries only. Do not merge RT bridge capture into H3 queue semantics without a new wave.

---

## 7. Anti-contamination checklist

| Risk | Mitigation |
|------|------------|
| Live telemetry in SA viewer | SA viewer static JSON only; no RT subscription |
| RT clock in compare mode | Compare bridge is replay-only (PLAT-SA-H4) |
| Federation pollution | No RT write path to `replay_federation_*` |
| Transient ID in lineage | Lint rejects `session_id` as authoritative parent |
| Auto corpus promotion | Forbidden; maintainer import only |
| Failed session promotion | Forbidden — see [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md) |

---

## 8. Gazebo runtime state and replay authority (PLAN-RT-G1)

| Concern | Rule |
|---------|------|
| Gazebo/ROS live state | **Transient** — not replay authority |
| Live telemetry | Explanatory mirrors only — not SA compare clocks |
| `capture_session` | Snapshots bridge-staged artifacts — not automatic SA import |
| Sim-only state | Omitted from SA bundles unless PLAT-RT-G5 normalization + conversion manifest |
| Runtime provenance | Explanatory fields only until G5; never `session_id` as lineage parent |

**Invariant (unchanged):** `capture_session ≠ SA replay import`.

---

## 9. Related

- [rt_capture_continuity_v1.md](rt_capture_continuity_v1.md)
- [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md)
- [rt_s1_governance_review_r1.md](rt_s1_governance_review_r1.md)
- [rt_gazebo_ros_boundary_v1.md](rt_gazebo_ros_boundary_v1.md)
- [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md)
