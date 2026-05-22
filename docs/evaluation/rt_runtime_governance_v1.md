# RT Runtime Governance (`rt_runtime_governance_v1`)

**Phase:** PLAN-RT-S1 — interactive runtime sandbox (docs only)  
**Authority:** [AGENTS.md](../../AGENTS.md); [experiment_orchestration_async_safety_v1.md](experiment_orchestration_async_safety_v1.md) (SA async boundary patterns)

Governance and safety rules for the **RT interactive sandbox** layer. Complements frozen SA platform governance; does not modify SA behavior.

---

## 1. Core principles

| Principle | RT-S1 rule |
|-----------|------------|
| Transient by default | Sessions are ephemeral; no implicit persistence |
| Deny-by-default | Bridge rejects unlisted commands |
| Mirrors ≠ authority | Telemetry mirrors are not replay truth |
| SA isolation | SA viewer and federation tooling cannot command RT runtime |
| No escalation | Failed or live sessions never auto-enter corpus/federation |

Required session banner (all RT UI surfaces):

`RT SANDBOX — experimental simulation; not operational state`

Secondary banner:

`TRANSIENT SESSION — not replay authority`

---

## 2. Authority matrix

| Surface | May start RT session? | May capture? | May mutate corpus/federation? |
|---------|----------------------|--------------|------------------------------|
| RT Browser UI (future) | Request via bridge only | Request `capture_session` only | **No** |
| RT Backend Bridge (future) | Yes (local prototype) | Emit capture candidate | **No** |
| `platform/sa-r0-viewer/` | **No** | **No** | **No** |
| `run_experiment_queue.py` | **No** (separate H3 path) | Via `--allow-runtime-capture` only | Via frozen SA CLIs after pack |
| Federation CLIs | **No** | **No** | **No** RT session input |

---

## 3. Prototype resource boundaries

Documented caps for RT-S2+ prototype implementation. Values are **governance constants** — changing them requires a new scoped wave and freeze audit row.

| Limit | Prototype default | Rationale |
|-------|-------------------|-----------|
| `max_entity_count` | **32** per session | Prevents fleet-scale / operational semantics |
| `max_concurrent_sessions` | **1** per bridge instance | No multi-tenant or distributed sandbox |
| `max_session_duration` | **3600 s** (60 min wall clock) | Prevents long-running “mission” sessions |
| `telemetry_update_rate_cap_hz` | **10 Hz** aggregate per session | Prevents live-dashboard / streaming infra drift |
| `command_rate_limit_burst` | **5 commands/s** | Bridge-side throttle |
| `command_rate_limit_sustained` | **1 command/s** | Bridge-side throttle |
| `session_cleanup_timeout` | **120 s** after `stop_session`, `discard_session`, or `failed` | Orphan process prevention |
| `cleanup_pending_max_age` | **300 s** | Forced teardown from `cleanup_pending` |
| `bridge_disconnected_reconnect_timeout` | **30 s** | Before `failed` + cleanup |
| `max_capture_bundle_bytes` | **5 MiB** | Per capture bundle (PLAT-RT-S5) |
| `max_staged_captures` | **32** | Staging dirs under `runs/rt_sandbox/captures/` |
| `max_runtime_templates_in_catalog` | **16** | Builtin RT template defs (PLAT-RT-S6) |
| `max_entities_per_template_apply` | **8** | Per `apply_runtime_template` |
| `max_template_applies_per_session` | **32** | Session-scoped applies |
| `max_workflows_in_catalog` | **8** | Builtin workflow defs |
| `max_workflow_steps` | **12** | Per workflow definition |

### 3.1 Limit enforcement

- Exceeding any cap → bridge returns `RESOURCE_LIMIT_EXCEEDED` (see [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md)).
- Session may transition to `failed` or `cleanup_pending` per [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md).
- Caps are **not** auto-tuned by load or user role in prototype phases.

### 3.2 Explicit non-goals (resource model)

Forbidden architectural directions for RT without a new wave:

- Horizontal scaling, session pooling, load balancing
- Multi-user collaboration rooms
- Cloud orchestration or remote bridge clusters
- Operational SLA / uptime semantics

---

## 4. Session isolation

- One **active** `running` or `paused` session per bridge instance.
- Telemetry subscriptions are **session-scoped** — no cross-session fan-in.
- Entity IDs are unique within session only; no global operational entity registry.
- `reset_session` clears session state; does not mutate SA fixtures.

---

## 5. Anti-escalation rules

| Path | Blocked |
|------|---------|
| Browser → ROS direct | All publish/subscribe |
| RT session → federation index | Live or batch writes |
| RT session → corpus lineage parent | `session_id` as `parent_ref` |
| `capture_session` → SA import | Without validation + packaging + audit |
| Failed session → replay bundle | Automatic promotion |
| Bridge admin bypass in UI | Hidden elevation to operational commands |

---

## 6. Replay integrity protection

- Only artifacts passing the SA export pipeline in [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md) may enter `replay_sa_bundle_v1` consumption.
- Captured bundles must declare `origin: rt_sandbox_capture_v1` and `parent_session_id` (ephemeral, explanatory only).
- SA viewer must not display RT live state or merge RT telemetry into compare/replay clocks.

---

## 7. RT copy and language rules

RT UX copy must remain **experimental**, **sandbox-oriented**, and **simulation-oriented**.

### 7.1 Forbidden terms (UI, docs, bridge messages)

`engage`, `intercept`, `strike`, `target lock`, `mission approval`, `operator authorization`, `tactical readiness`, `command authority`, `defeat`, `kill`, `neutralize` (operational sense), `fielding`, `go-live`, `certification` (affirmative operational voice).

### 7.2 Preferred alternatives

| Avoid | Prefer |
|-------|--------|
| engage / intercept | simulate action, reposition entity |
| target lock | select / highlight entity |
| mission approval | session control |
| tactical readiness | sandbox status |

Future RT lint may extend `governance_lint_sa.py` patterns — RT-S1 documents list only.

### 7.3 Capture UI banner (future)

`CAPTURE CANDIDATE — requires validation before replay import`

---

## 8. Bridge security summary

Full assumptions: [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md) § Prototype bridge security assumptions.

| Assumption | RT-S1 stance |
|------------|--------------|
| Network | Local-only (loopback / unix socket) during RT-S2–S4 |
| Browser credentials | Never stores ROS/DDS tokens |
| Bridge policy | Deny-by-default allow-list |
| Production auth | Out of scope (no OAuth/RBAC/mTLS in RT-S1) |

---

## 9. Auditability

All bridge commands (future) append to `rt_session_audit_log_v1`:

- `command_id`, `session_id`, `issued_by`, `command_type`, `result`, `governance_banner`

Audit logs are explanatory; not parser contracts or federation authority.

---

## 10. Related

- [rt_session_lifecycle_v1.md](rt_session_lifecycle_v1.md)
- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md)
- [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)
- [rt_s1_governance_review_r1.md](rt_s1_governance_review_r1.md)
