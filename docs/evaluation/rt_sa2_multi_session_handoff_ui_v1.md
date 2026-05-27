# RT SA2 Multi-Session Handoff UI Contract (`rt_sa2_multi_session_handoff_ui_v1`)

**Phase:** PLAT-RT-SA2 — multi-session RT→SA workflow UX  
**Authority:** [rt_sa2_multi_session_handoff_workflow_plan.md](../platform/rt_sa2_multi_session_handoff_workflow_plan.md); [rt_sa_import_bridge_v1.md](rt_sa_import_bridge_v1.md); [rt_runtime_workstation_ui_v1.md](rt_runtime_workstation_ui_v1.md)

Additive supplement for read-only handoff staging mirror and workstation UI. Does not replace frozen PLAT-RT-SA1 import semantics.

---

## 1. Core invariant

`list_capture_handoff_status ≠ SA replay import`. Browser mirror is **explanatory** only. SA lineage begins only on explicit maintainer `rt_sa_import commit`.

RT authority ends at handoff staging; SA lineage starts at corpus commit.

---

## 2. Bridge command: `list_capture_handoff_status`

| Field | Rule |
|-------|------|
| Transport | `POST /v1/command` (loopback) |
| Payload | `{ "session_id": "<uuid>" }` **required** |
| Session in registry | **Not** required — captures may outlive evicted sessions |
| Writes | **None** — read filesystem only |
| Rate limit | Global command rate limit |

**Response (success):**

| Field | Type |
|-------|------|
| `captures` | `rt_capture_handoff_row_v1[]` |
| `session_id` | string (echo filter) |
| `governance_banner` | HANDOFF MIRROR — read-only; not SA replay authority |

Empty `captures` when no staged rows match — not an error.

### 2.1 Isolation

Handler MUST return only captures where `candidate.session_id` or `candidate.ephemeral_session_ref` equals requested `session_id`. Cross-session leakage is forbidden.

### 2.2 SA1 boundary

This command is **not** an import command. PLAT-RT-SA1 forbade bridge HTTP commands for **import**; read-only mirror is authorized in PLAT-RT-SA2.

---

## 3. Row schema: `rt_capture_handoff_row_v1`

| Field | Required | Notes |
|-------|----------|-------|
| `capture_candidate_id` | yes | Cross-ref only |
| `approval_status` | yes | `pending` / `approved` / `rejected` |
| `normalization_status` | yes | `pending` / `normalized` / `rejected` |
| `handoff_decision` | no | From `handoff_review.decision` |
| `workflow_phase` | yes | See §4 |
| `validation_ok` | yes | From `validate_normalized_capture` |
| `validation_errors` | no | Bounded list (max 8 strings) |
| `has_handoff_manifest` | yes | `sa_handoff/<id>/handoff_manifest.json` |
| `has_import_record` | yes | Committed cognition |
| `last_export_event_type` | no | Latest relevant export event |
| `source_origin` | yes | `rt_sandbox_capture_v1` |
| `lineage_note` | yes | Session ref non-authoritative |
| `governance_banner` | yes | Per-row mirror banner |

**Forbidden in rows:** absolute corpus paths, full `candidate.json`, SA bundle authority fields.

---

## 4. `workflow_phase` derivation

| Phase | Condition |
|-------|-----------|
| `committed` | `import_record.json` exists under `sa_handoff/<id>/` |
| `rejected` | `handoff_decision == rejected` OR `approval_status == rejected` |
| `deferred` | `handoff_decision == deferred` |
| `prepared` | `handoff_manifest.json` exists |
| `ready` | Normalized + preconditions empty + `approval_status == approved` + review `ready_for_approval` |
| `review_pending` | Normalized; not ready/blocked |
| `normalized` | `normalization_status == normalized` |
| `staged` | Staging dir + `candidate.json` exist |
| `none` | Fallback |

---

## 5. UI requirements (PLAT-RT-SA2)

### 5.1 Governance banners

Add persistent banner: **`MANUAL HANDOFF ONLY`** (in `BASE_BANNERS`).

Existing banners unchanged: RT SANDBOX, TRANSIENT RUNTIME ONLY, NOT SA REPLAY AUTHORITY.

### 5.2 Workstation zones

| Zone | SA2 addition |
|------|----------------|
| Pipeline footer | Per-session capture table + multi-session overview |
| Session tabs | Badge when captures exist or phase `ready` |
| Background diagnostics | `captures=N`, `handoff=<phase>` per slot |

### 5.3 Polling

- Poll `list_capture_handoff_status` per connected session id at ≤ **1 Hz**
- Store per-session rows in UI state map

### 5.4 Forbidden in browser

- `capture_session`, `rt_sa_import`, corpus writes
- SA viewer imports
- Auto-run import pipeline steps

---

## 6. Maintainer helpers

UI may show CLI copy chips and static pipeline steps from [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md).

Optional CLI: `rt_capture_inspect list --session-id <uuid>` using same session filter as bridge.

---

## 7. Explicit non-goals

- SA viewer integration
- Automatic replay import
- Federation UI
- Browser corpus authority
- Distributed runtime
- New telemetry channels

---

## 8. T4 boundary (additive override)

PLAT-RT-T4 §8 stated “no staging API.” PLAT-RT-SA2 **adds** read-only `list_capture_handoff_status` only. All other T4 non-goals remain.

---

## Related

- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) §12 (PLAT-RT-SA2 additive)
- [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md)
- [rt_multi_session_governance_v1.md](rt_multi_session_governance_v1.md)
