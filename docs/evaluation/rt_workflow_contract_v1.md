# RT Workflow Contract (`rt_workflow_contract_v1`)

**Phase:** PLAT-RT-S6 — sandbox templates and multi-step runtime workflows  
**Schema version:** `rt_workflow_contract_v1`  
**Authority:** [rt_bridge_contract_v1.md](rt_bridge_contract_v1.md); [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md)

Session-scoped, transient workflow and template semantics for the RT interactive sandbox bridge. Templates are **not** SA scenario corpus lineage.

---

## 1. Runtime templates (`rt_runtime_template_v1`)

| Field | Required | Description |
|-------|----------|-------------|
| `template_id` | yes | Builtin id (suffix `_v1`) |
| `kind` | yes | `radar_preset`, `drone_preset`, `waypoint_layout`, `world_init` |
| `entity_count` | yes | Entities spawned on apply (0 for `world_init` metadata) |
| `description` | yes | Prototype-only label |
| `entities` | when count > 0 | List of `{entity_type, pose}` |

Templates live in bridge code catalog only. No writes to `fixtures/scenarios/`.

---

## 2. Template commands

| Command | Preconditions | Effect |
|---------|---------------|--------|
| `list_runtime_templates` | none or active session | Return catalog metadata |
| `apply_runtime_template` | `running` or `paused` | Spawn preset entities |

**Payload (`apply_runtime_template`):** `{ "template_id": "<id>" }`

---

## 3. Workflows (`rt_sandbox_workflow_v1`)

| Field | Description |
|-------|-------------|
| `workflow_id` | Builtin workflow id |
| `current_step` | 0-based index |
| `step_count` | Total steps |
| `status` | `idle`, `in_progress`, `completed`, `failed` |

### Workflow commands

| Command | Preconditions | Effect |
|---------|---------------|--------|
| `start_workflow` | `running`/`paused`; workflow idle | Begin at step 0 |
| `advance_workflow` | workflow `in_progress` | Execute one step |
| `reset_workflow` | active session | Clear workflow state |
| `reload_workflow` | active session | Reset + restart same `workflow_id` |
| `get_workflow_state` | active session | Read-only summary |

**Payload (`start_workflow` / `reload_workflow`):** `{ "workflow_id": "<id>" }`

### Step kinds

| Step kind | Behavior |
|-----------|----------|
| `reset_world` | Clear session entities (same as internal reset) |
| `apply_template` | Apply `template_id` from step |
| `ready` | Mark workflow completed |

On step failure → `status: failed`, error `WORKFLOW_STEP_FAILED`.

### Adapter resync (PLAT-RT-R2d)

After `apply_template` or `reset_world` workflow steps (and direct `apply_runtime_template`), the bridge runs [rt_template_resync_policy_v1.md](rt_template_resync_policy_v1.md): automatic adapter resync or sim clear when the Gazebo adapter is active. `reset_workflow` / `reload_workflow` do not trigger resync.

---

## 4. Audit extensions

Append to `rt_session_audit_log_v1` with additive `detail`:

- `template_id`, `entities_spawned`, `revision` on template apply
- `workflow_id`, `step_index`, `transition`, `staged_setup` on workflow steps

---

## 5. Blocked (always)

| Category | Examples |
|----------|----------|
| SA import | `import_scenario`, `import_replay`, `auto_capture` |
| Corpus | `save_template_to_corpus`, `promote_workflow` |
| Orchestration | `orchestration_apply` |

---

## 6. Capture metadata (additive)

`runtime_capture_report_v1` may include:

- `workflow_summary` — final workflow state at capture
- `templates_applied` — list of template ids applied in session

No automatic `scenario_pack_ref` from workflow.
