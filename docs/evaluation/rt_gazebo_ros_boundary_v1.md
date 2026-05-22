# RT ↔ Gazebo/ROS Boundary (`rt_gazebo_ros_boundary_v1`)

**Phase:** PLAN-RT-G1 — Gazebo/ROS integration boundary (docs only)  
**Schema version:** `rt_gazebo_ros_boundary_v1`  
**Authority:** [rt_g1_gazebo_ros_integration_plan.md](../platform/rt_g1_gazebo_ros_integration_plan.md); [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md)

Normative boundary between **RT Backend Bridge**, **RT Runtime Adapter** (future), and **Gazebo/ROS2** simulation engine. No adapter or ROS implementation in RT-G1.

---

## 1. Architectural placement

```mermaid
flowchart LR
  Bridge[RT_Backend_Bridge]
  Adapter[RT_Runtime_Adapter]
  ROS[ROS2_graph]
  GZ[Gazebo]

  Bridge -->|session_RPC_deny_default| Adapter
  Adapter -->|allow_list_only| ROS
  ROS --> GZ
```

| Component | Process model | Credentials |
|-----------|---------------|-------------|
| Bridge | Long-lived local process | None exposed to browser |
| Adapter | Per-session child (G2+) | ROS/DDS local to adapter only |
| Gazebo | Adapter-managed child | Not browser-accessible |

---

## 2. Allowed RT ↔ ROS interactions (future waves)

All paths require an **active RT session** and **running or paused** lifecycle state unless noted.

| Path | Direction | Allowed when | Forbidden always |
|------|-----------|--------------|------------------|
| Session start | Bridge → Adapter | `start_session` / `created` → `running` | Browser → ROS |
| Entity pose apply | Bridge → Adapter → ROS | G3+; command-authoritative bridge pose | Direct UI publish |
| Entity mirror read | ROS → Adapter → Bridge | G4+; read-only telemetry channels | SA viewer subscribe |
| Clock pause | Bridge → Adapter | `pause_session` / `send_runtime_command` allow-list | Wall-clock as replay authority |
| World reset | Bridge → Adapter | `reset_session` | Corpus world promotion |
| Session stop | Bridge → Adapter | `stop_session`, `discard_session`, cleanup | Partial orphan leave-behind |
| Telemetry drain | Bridge internal | Existing PLAT-RT-S4 pull path | WebSocket in SA viewer |

**PLAT-RT-S2–S6 (current):** No row in this table is live except bridge-internal mirrors from `WorldStateStore` and `RuntimeStub`. Adapter column is **documentation only**.

---

## 3. Blocked authority paths

| Path | Block reason | Error surface (bridge) |
|------|--------------|------------------------|
| Browser → ROS publish/subscribe | No browser credentials | `COMMAND_FORBIDDEN` |
| Browser → Gazebo API | No sim API in UI | `COMMAND_FORBIDDEN` |
| Bridge → corpus/federation | SA-only authority | `COMMAND_FORBIDDEN` |
| Bridge → orchestration queue | H3 separate entrypoint | `COMMAND_FORBIDDEN` |
| Adapter → SA import | Export boundary | Adapter internal reject |
| Adapter → `fixtures/scenarios/` write | Authoring is SA path | Adapter internal reject |
| RT session → `/tracks/state` mutation | Parser contract freeze | `COMMAND_FORBIDDEN` |
| RT session → weapon/engage topics | Operational semantics | `COMMAND_FORBIDDEN` |
| rosbridge / legacy `web/` | Frozen SA/RT boundary | Not used for RT |
| Distributed DDS / remote master | No distributed infra | Launch config blocked in G2 audit |

---

## 4. RT Runtime Adapter responsibilities (G2+)

| Responsibility | Owner | Notes |
|----------------|-------|-------|
| Spawn/teardown Gazebo + ROS graph for session | Adapter | Bridge issues lifecycle commands only |
| Enforce ROS topic allow-list | Adapter | Deny-by-default; bridge does not publish to ROS directly |
| Map bridge `entity_id` ↔ `sim_entity_ref` | Adapter | Session-scoped; cleared on reset/stop |
| Propagate sim clock mirror | Adapter → Bridge | Non-authoritative; capped 10 Hz aggregate |
| Report adapter health | Adapter → Bridge | `session_health.stub_alive` evolves to `adapter_alive` in G2 |
| Orphan process cleanup | Adapter + Bridge | Bridge `cleanup_pending` supervises kill tree |
| Capture file write | Bridge | Adapter may supply snapshots; bridge stages capture |

Adapter **must not:**

- Call SA packager, federation CLIs, or `run_experiment_queue.py`
- Auto-set `scenario_pack_ref` on capture
- Persist sim state across sessions

---

## 5. Topic ownership model

### 5.1 Principles

1. **Deny-by-default:** No ROS publish/subscribe until PLAT-RT-G2+ wave extends allow-list in freeze audit.
2. **Session prefix:** RT session topics use a declared prefix (e.g. `/rt_sandbox/<session_id>/...`) — exact names fixed in G2 audit, not RT-G1.
3. **Transient scope:** Topics and nodes die with adapter teardown; no cross-session DDS graph sharing.
4. **Parser freeze:** Evaluation parser topics (including `/tracks/state`) are **read-only mirrors at most** — never mutated from RT path.

### 5.2 Topic category table (stub — not live enumeration)

| Category | RT adapter role | SA replay authority |
|----------|-----------------|---------------------|
| `rt_session_control` | Bridge→Adapter commands (private IPC preferred over ROS) | No |
| `rt_entity_pose_cmd` | Bridge-authoritative pose apply (G3) | No |
| `rt_entity_pose_mirror` | Read-only mirror to bridge telemetry (G4) | No |
| `rt_clock_mirror` | Read-only sim pause state | No |
| `parser_contract_topics` | **Blocked** for publish; optional read-only mirror with governance audit | **Yes** (offline replay only) |
| `weapon_operational` | **Blocked** | No |
| `federation_orchestration` | **Blocked** | No |

### 5.3 Forbidden subscriptions (unchanged from PLAT-RT-S4)

Including but not limited to: `/tracks/state` as live operational truth, engage/intercept channels, federation status, tactical/readiness channels.

---

## 6. Transient runtime scope

| Object | Lifetime | Persistence |
|--------|----------|-------------|
| `session_id` | Bridge session | None |
| `sim_entity_ref` | Adapter mapping | Cleared on reset/stop/cleanup |
| ROS nodes launched by adapter | Session | Killed on cleanup |
| Gazebo world instance | Session | Destroyed on cleanup |
| Capture staging dir | Until maintainer export or purge | Under `runs/rt_sandbox/captures/` only |

---

## 7. Failure and cleanup authority

| Event | Detection | Cleanup authority | Bridge state |
|-------|-----------|-------------------|--------------|
| Gazebo launch failure | Adapter exit code / timeout | Adapter teardown + bridge `failed` | `cleanup_pending` |
| ROS node crash | Adapter watchdog | Kill adapter subtree | `runtime_crashed` |
| Topic timeout | Adapter mirror stall | Unsubscribe + optional `failed` | Degraded or `failed` |
| Stale entity sync | Pose drift threshold (G3) | `reset_session` or resync | `running` or `INVALID_POSE` |
| Adapter disconnect | IPC heartbeat loss | Orphan kill after `bridge_disconnected_reconnect_timeout` | `failed` |

**Replay authority during all failures:** none. Failure blobs in audit log are explanatory only.

Detail: [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md) § 6.

---

## 8. `send_runtime_command` adapter profile (deferred)

Bridge command `send_runtime_command` remains **deny-by-default** until PLAT-RT-G2 documents sub-commands.

Illustrative future sub-commands (not active in RT-G1):

| Sub-command | Phase | Notes |
|-------------|-------|-------|
| `adapter_attach` | G2 | Start adapter + sim; maintainer-gated in prototype |
| `adapter_detach` | G2 | Tear down without full session stop |
| `set_clock_pause` | G2/G3 | Prefer `pause_session` |
| `reload_world_config` | G2+ | Maintainer-only |

See additive § in [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md).

---

## Related

- [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md)
- [rt_runtime_synchronization_v1.md](rt_runtime_synchronization_v1.md)
- [rt_sa_export_boundary_v1.md](rt_sa_export_boundary_v1.md)
