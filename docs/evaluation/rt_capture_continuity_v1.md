# RT Capture Continuity (`rt_capture_continuity_v1`)

**Phase:** PLAT-RT-S5 — schemas implemented in `platform/rt-sandbox-bridge/rt_sandbox/capture.py`  
**Authority:** [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)

Defines continuity artifacts for RT sandbox sessions and replay-import safety. Implementation: PLAT-RT-S5; SA import remains maintainer-gated.

---

## 1. Artifact overview

| Schema | Role | SA viewer import |
|--------|------|------------------|
| `sandbox_session_snapshot_v1` | RT-local debug / resume within sandbox | **Blocked** |
| `runtime_capture_report_v1` | Command audit + timing explanatory report | Only with bundle via export pipeline |
| `runtime_to_replay_conversion_v1` | Maps snapshot + logs → bundle inputs | Required for RT-origin SA packs |

---

## 2. `sandbox_session_snapshot_v1`

**Purpose:** Optional checkpoint while session is `paused` or `stopped` — **RT-internal only**.

```json
{
  "schema": "sandbox_session_snapshot_v1",
  "session_id": "uuid-ephemeral",
  "snapshot_utc": "ISO-8601",
  "entity_states": [],
  "governance_banner": "RT SNAPSHOT — non-authoritative; not replay truth"
}
```

**Rules:**

- Not parser-visible
- Not federation-visible
- Must not be loaded by `platform/sa-r0-viewer/`
- Failure states (`failed`, `runtime_crashed`) may produce snapshot for debug — still non-authoritative

---

## 3. `runtime_capture_report_v1`

**Purpose:** Explanatory audit companion to capture candidate.

```json
{
  "schema": "runtime_capture_report_v1",
  "capture_candidate_id": "uuid",
  "session_id": "uuid-ephemeral",
  "command_summary": [],
  "resource_limit_events": [],
  "failure_states_observed": [],
  "governance_banner": "CAPTURE REPORT — explanatory; not operational assessment"
}
```

Pairs with `rt_session_audit_log_v1` from [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md).

---

## 4. `runtime_to_replay_conversion_v1`

**Purpose:** Declares how RT staging artifacts become SA bundle inputs (RT-S5).

```json
{
  "schema": "runtime_to_replay_conversion_v1",
  "capture_candidate_id": "uuid",
  "scenario_pack_ref": "fixtures/scenarios/...",
  "log_path": "runs/logs/....log",
  "conversion_steps": [
    "validate_scenario_pack",
    "replay_observability",
    "replay_sa_bundle_pack",
    "governance_lint"
  ],
  "origin": "rt_sandbox_capture_v1",
  "governance_banner": "CONVERSION MANIFEST — maintainer pipeline only"
}
```

---

## 5. Replay-import safety requirements

Import into SA replay consumption **must** satisfy:

1. `explicit_approval_record` present per [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)
2. `origin` includes `rt_sandbox_capture_v1`
3. `runtime_to_replay_conversion_v1` manifest committed with bundle
4. No raw live telemetry blobs without conversion declaration
5. `session_id` not used as lineage authority parent
6. Governance lint pass on packaged bundle
7. **No** automatic federation index or publication collection update

Failed or `cleanup_pending` sessions **must not** produce importable conversion manifests.

---

## 6. Failure state interaction

See [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md) § Failure vs capture.

| State | Snapshot allowed? | Capture report? | Conversion manifest? |
|-------|-------------------|-----------------|----------------------|
| `stopped` | Optional | On `capture_session` | After approval only |
| `captured` | N/A | Yes | Pending pipeline |
| `failed` | Debug only | Explanatory | **Forbidden** |
| `discarded` | **No** | **No** | **No** |

---

## 7. Related

- [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)
- [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md)
- [rt_roadmap_s2_s6_v1.md](rt_roadmap_s2_s6_v1.md) — RT-S5 implementation wave
