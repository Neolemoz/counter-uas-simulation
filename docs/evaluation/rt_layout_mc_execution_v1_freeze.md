# Layout MC Execution V1 — Freeze Audit

**Phase:** Layout MC Execution (Steps 1–5)  
**Registry:** `PLAT-RT-LAYOUT-MC-EXEC1`  
**Status:** frozen (maintainer CLI only)

**Contract:** [rt_layout_monte_carlo_execution_v1.md](rt_layout_monte_carlo_execution_v1.md)  
**Upstream:** [rt_layout_monte_carlo_v1.md](rt_layout_monte_carlo_v1.md) (`rt_layout_mc_profile.py`, layout handoff export in UI)  
**Adjacent (separate freeze):** [planning_mc_integration_v2.md](planning_mc_integration_v2.md) (Planning UI linkage; no execution)

---

## Architecture overview

Layout MC Execution V1 is **offline maintainer plumbing** between RT layout handoff JSON and the existing Phase 4 Monte Carlo harness (`scripts/monte_carlo.py` → `scripts/run_capture.py`).

```
Browser handoff JSON (rt_layout_mc_handoff_v1)
        │
        ▼
  prepare ──► runs/rt_sandbox/mc_jobs/<job_id>/
        │         manifest.json, status.json, command.txt
        ▼
  execute ──► subprocess: monte_carlo.py run …
        │         runs/mc/<job_id>.json|.csv
        ▼
  post-validate + handoff artifacts
        │         result_summary.json, result_link.json
        ▼
  planning-ref map (optional, stdout)
        └──► rt_planning_mc_result_ref_v1 paste payload
```

No RT bridge HTTP surface, no browser execution path, and no automatic Planning UI import.

---

## Execution lifecycle

| Status | Meaning |
|--------|---------|
| `prepared` | `prepare` wrote job dir; MC not started |
| `running` | `execute` invoked subprocess |
| `completed` | subprocess OK + output validation + handoff artifacts written |
| `failed` | non-zero exit and/or validation failure |

CLI commands:

| Command | Role |
|---------|------|
| `prepare` | Validate handoff/preview; write manifest, status, command preview |
| `status` | Print `status.json` |
| `render-command` | Print rendered MC command |
| `execute` | Run MC command; update status (optional `--dry-run`) |

`rt_layout_mc_planning_ref.py map` emits Planning-compatible result-ref JSON from completed job artifacts (maintainer paste only).

---

## Artifact flow

| Artifact | Schema | Written by |
|----------|--------|------------|
| `manifest.json` | `rt_layout_mc_execution_manifest_v1` | `prepare` |
| `status.json` | `rt_layout_mc_job_status_v1` | `prepare` / `execute` |
| `command.txt` | — | `prepare` |
| `result_summary.json` | `rt_layout_mc_result_summary_v1` | `execute` (success) |
| `result_link.json` | `rt_layout_mc_result_link_v1` | `execute` (success) |
| `runs/mc/<job_id>.json` | MC aggregate summary | `monte_carlo.py` |
| `runs/mc/<job_id>.csv` | per-run table | `monte_carlo.py` |

---

## Identifier propagation

Aligned across manifest, completed status, `result_summary.json`, and `result_link.json`:

| Field | Role |
|-------|------|
| `job_id` | MC `--label`; maps to Planning `mc_run_label` when pasted |
| `geometry_id` | `rt_layout:sha256:*` layout fingerprint |
| `source_layout_id` | layout lineage hint |
| `cohort` | evaluation cohort tag for `run_capture` / aggregate filters |

`audit_layout_identifier_propagation()` in `rt_layout_mc_planning_ref.py` detects cross-artifact drift.

**Not propagated automatically:** `planning_snapshot_id`, `planning_geometry_id` (`rt_planning:*` namespace).

---

## Planning linkage path

Separate from Grid layout handoff UI export and from Planning MC Integration V2 UI modules.

1. Maintainer completes layout MC job (`execute` success).
2. Maintainer runs `rt_layout_mc_planning_ref.py map` with explicit `--linked-package-id` and `--linked-planning-geometry-id`.
3. Maintainer pastes emitted `rt_planning_mc_result_ref_v1` into Planning UI (existing import path).

Golden crosswalk: [rt_layout_mc_planning_result_ref_golden_v1.json](../../fixtures/rt_sandbox/rt_layout_mc_planning_result_ref_golden_v1.json) paired with [planning_mc_result_link_golden_v1.json](../../fixtures/rt_sandbox/planning_mc_result_link_golden_v1.json) Planning ids.

No `planning_result_link_v1` auto-build, no filesystem load in Planning UI, no schema changes in this wave.

---

## Governance audit (Step 5)

| Check | Verdict |
|-------|---------|
| Maintainer-only execution | **Pass** — CLI under `scripts/evaluation/`; no UI execute button |
| CLI-only execution | **Pass** — `prepare`, `status`, `render-command`, `execute`, `planning-ref map` |
| No UI execution | **Pass** — browser handoff remains export-only (`layoutMcHandoff.ts`) |
| No bridge coupling | **Pass** — no `platform/rt-sandbox-bridge/` imports; tests assert no `rt_bridge` / `rosbridge` in planning-ref module |
| No runtime mutation | **Pass** — no RT session commands; MC via existing offline harness only when maintainer runs `execute` |
| No SA promotion | **Pass** — outputs under `runs/mc/` and `runs/rt_sandbox/mc_jobs/` only |
| No Planning auto-import | **Pass** — mapping helper prints JSON; no Planning TS module changes |
| No automatic result authority | **Pass** — MC summaries are descriptive simulation stats; handoff artifacts are maintainer review mirrors |

---

## Limitations

- Requires `install/setup.bash` and maintainer shell for real `execute` (Gazebo/ROS).
- No job queue, cancel, distributed execution, or concurrent-run locks.
- `geometry_id` (`rt_layout:*`) is not interchangeable with `planning_geometry_id` without explicit maintainer alignment.
- MC summary metrics in `result_summary.json` are a lightweight subset (not full per-run tables).
- Re-execute from `completed` is blocked; retry allowed from `failed` only.
- No automatic linkage into Planning Layout Comparison slots.

---

## Future work (not authorized by this freeze)

- UI job status mirror (read-only) in RT sandbox workstation.
- `validate-handoff` / `cancel` CLI ergonomics.
- Optional `planning_result_link_v1` shell export (still paste-only).
- Scoped bridge between Planning package export and layout handoff when maintainer pain warrants a new PLAN wave.

---

## Regression evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_layout_mc_execute.py -q
git diff --check
```

Step 5 validation (2026-06-05): **25 passed**; `git diff --check` clean.

---

## Freeze verdict

**Frozen** as **PLAT-RT-LAYOUT-MC-EXEC1** — Layout MC Execution V1 maintainer CLI (prepare → execute lifecycle → result handoff → optional Planning result-ref mapping). Hold boundary: no UI execution, no bridge/protocol changes, no SA promotion, no Planning schema or auto-import changes without a new scoped PLAN + governance audit.
