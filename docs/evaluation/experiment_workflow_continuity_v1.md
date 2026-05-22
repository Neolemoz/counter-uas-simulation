# Experiment Workflow Continuity — Viewer Navigation (v1)

**Phase:** PLAT-SA-H4 — workstation integration  
**Extends:** [experiment_workflow_scenario_to_replay_v1.md](experiment_workflow_scenario_to_replay_v1.md)

## Reviewer path (browser)

1. **Scenario** — pick topology pack; validation + queue mirrors are read-only.
2. **Replay** — bundle loads; lineage panel links hops; optional sweep workstation.
3. **Compare** — segment always shows pair catalog; curated pairs load A/B replay.
4. **Corpus** — browser + lineage nav to linked demo/sweep/presentation.
5. **Report** — storyboard picker and bundle walkthrough entry.

CLI/CI still runs `run_experiment_queue.py`; the viewer never launches simulation.

## URL param precedence

| Param | Effect |
|-------|--------|
| `corpus_entry` | Resolves corpus entry first (demo/sweep/presentation) |
| `sweep` | Sweep workstation when no corpus override |
| `pair` / `compare` | Compare mode |
| `presentation` / `walkthrough` | Presentation mode |
| `demo` | Single replay bundle |
| `orchestration_queue` | Selected queue mirror id (explanatory) |

Entering compare clears `demo` and sets `pair` or `compare`. Orchestration queue selection updates `orchestration_queue` only.

## Deep links from mirrors

Queue job `provenance` fields navigate when resolvable:

- `scenario_pack_id` → load demo bundle, Replay segment
- `bundle_path` → resolve pack via catalog slug
- `corpus_ref` → corpus entry `demo_bundle__{ref}` when present in index

## Related

- [h4_sandbox_replay_workstation_integration_plan.md](../platform/h4_sandbox_replay_workstation_integration_plan.md)
- [sa_h4_workstation_integration_freeze_audit.md](sa_h4_workstation_integration_freeze_audit.md)
