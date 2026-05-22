# Experiment Orchestration Async Safety (`experiment_orchestration_async_safety_v1`)

**Phase:** PLAN-SA-I2 — async safety boundaries (docs only)  
**Authority:** [AGENTS.md](../../AGENTS.md); [sa_i2_async_orchestration_plan.md](../platform/sa_i2_async_orchestration_plan.md)

Hard **safety boundaries** for any future async orchestration. Violations require a new scoped wave and governance review — not silent extension.

---

## 1. Authority matrix

| Surface | May execute orchestration? | May mutate queues/ops? | May promote lifecycle? |
|---------|---------------------------|------------------------|------------------------|
| CLI / future worker | Yes (when PLAT-SA-I2+ authorized) | Yes (via frozen contracts) | Yes (CLI only) |
| CI (bounded) | Yes, with explicit flags only | Yes, offline lane only | Yes, via promote CLI |
| Viewer / browser | **No** | **No** | **No** |
| Parser / evaluation summaries | **No** | **No** | **No** |
| Mirrors (`public/demo/orchestration/`) | **No** | **No** | **No** |

**Mirrors ≠ authority.** Governance banners remain mandatory on all orchestration mirror panels.

---

## 2. Hard prohibitions (I2.2)

Each prohibition applies to PLAN-SA-I2 docs, future PLAT-SA-I2+ implementation, and any CI worker lane unless a later wave explicitly reopens with audit.

### 2.1 No browser execution authority

Forbidden in viewer or eval web tooling:

- Invoking `run_experiment_queue.py` from UI events
- Calling `promote_experiment_manifest.py` from UI
- Spawning or signaling async workers from browser code
- `launch_action`, `execute_queue`, or equivalent engagement controls

### 2.2 No runtime streaming into viewer

Forbidden:

- Live `/tracks/state` or fused detection streams in SA-R0 viewer
- rosbridge / WebSocket eval paths in `platform/sa-r0-viewer/`
- Extending legacy `web/` rosbridge pages for orchestration status
- Progress streams that imply operational command or live mission control

Replay remains **static JSON / bundle consumption** only.

### 2.3 No nondeterministic replay generation

Forbidden unless manifest declares seed policy **and** records fingerprint:

- Undeclared random seeds in capture pipelines
- Wall-clock–coupled bundle identity without documented tolerance
- Environment-specific paths that change bundle hashes without lineage record

Default: matched-seed, fingerprinted, reproducible artifacts per I1 repro-check patterns.

### 2.4 No hidden queue mutation

Forbidden:

- In-place edits to published `experiment_run_queue_v1` snapshots
- Viewer refresh that rewrites queue files
- Worker writes that bypass audit report generation
- Silent supersession without `superseded` lineage marker

All queue evolution: new snapshot + report + optional claim record.

### 2.5 No live operational semantics

Forbidden framing (use governance-safe negatives in copy):

- Mission scheduling, command and control, tactical dashboards
- Deployment readiness, operational effectiveness, recommend intercept
- HITL / operator workflow semantics
- Realtime orchestration service or live mission scheduler identity

Platform identity remains **governance-aware replay experimentation** — not operational command infrastructure.

---

## 3. Governance notice template (future artifacts)

Future async artifacts should use:

```json
{
  "governance": {
    "notice": "Explanatory orchestration mirror only. CLI and workers are authoritative. Not operational readiness.",
    "anti_claims": [
      "not live mission control",
      "not browser-triggered execution",
      "not parser-visible authority"
    ]
  }
}
```

---

## 4. Future-safe viewer posture (I2.7)

Frozen I1 panels remain the baseline:

- `OrchestrationLifecyclePanel` — `operations_status` explanatory only
- `OrchestrationIntegrityPanel` — integrity report mirror
- `OrchestrationReplayContinuityPanel` — bundle/corpus provenance

**Required posture for any future async UI (PLAT-SA-I2+):**

| Property | Requirement |
|----------|-------------|
| Mode | Explanatory-only, read-only, replay-oriented |
| Authority | Non-authoritative; mirrors refreshed by CLI sync only |
| Controls | **No** orchestration control UI, worker dashboard, or queue launch |
| Live data | **No** WebSocket progress, live ROS, or streaming step updates |
| Allowed additions | Claim status mirror, `async_execution_status` badge, integrity flags, governance banners — same pattern as existing integrity panel |

URL params (`orchestration_queue`, `orchestration_manifest`) remain navigation hooks only; they do not grant execution authority.

---

## 5. Architecture split (unchanged)

| Layer | Role |
|-------|------|
| Web platform | Scenario authoring + orchestration **planning/ops** + replay/research ecosystem |
| Gazebo/ROS2 | Runtime simulation engine (external to viewer) |

Async workers execute **outside** the viewer; the viewer never becomes a runtime orchestration client.

---

## Related

- [experiment_orchestration_async_model_v1.md](experiment_orchestration_async_model_v1.md)
- [experiment_orchestration_async_governance_v1.md](experiment_orchestration_async_governance_v1.md)
- [experiment_orchestration_operations_v1.md](experiment_orchestration_operations_v1.md)
- [sa_i1_orchestration_operations_freeze_audit.md](sa_i1_orchestration_operations_freeze_audit.md)

*End of experiment orchestration async safety v1.*
