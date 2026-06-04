# RT Layout Monte Carlo Execution V1

## Scope

`rt_layout_mc_execution_manifest_v1` is maintainer-only offline plumbing for
preparing Monte Carlo execution from exported RT layout/job handoff JSON.

This path does not run Monte Carlo by default. It writes a manifest, a prepared
status artifact, and a command preview that a maintainer may run separately in a
controlled shell.

## Inputs

Supported inputs:

- `rt_layout_mc_handoff_v1`
- `rt_mc_job_preview_v1`

The combined handoff is preferred because it carries both the layout and prepared
job. The CLI can recompute the layout geometry fingerprint and warn when the
prepared job geometry is stale.

## CLI

Prepare a job directory:

```bash
python3 scripts/evaluation/rt_layout_mc_execute.py prepare \
  fixtures/rt_sandbox/rt_layout_mc_handoff_golden_v1.json \
  --seed-base 7001 \
  --cohort rt_layout_mc_review
```

Prepared artifacts are written under:

```text
runs/rt_sandbox/mc_jobs/<job_id>/
```

Files:

- `manifest.json`
- `status.json`
- `command.txt`

Inspect prepared status:

```bash
python3 scripts/evaluation/rt_layout_mc_execute.py status \
  runs/rt_sandbox/mc_jobs/<job_id>
```

Render the dry-run command:

```bash
python3 scripts/evaluation/rt_layout_mc_execute.py render-command \
  runs/rt_sandbox/mc_jobs/<job_id>
```

Execute (maintainer shell; updates job status):

```bash
python3 scripts/evaluation/rt_layout_mc_execute.py execute \
  runs/rt_sandbox/mc_jobs/<job_id>

python3 scripts/evaluation/rt_layout_mc_execute.py execute \
  runs/rt_sandbox/mc_jobs/<job_id> --dry-run
```

## Manifest

The manifest schema is `rt_layout_mc_execution_manifest_v1`.

Fields:

- `job_id`
- `geometry_id`
- `source_layout_id`
- `run_count`
- `seed_base`
- `scenario`
- `launch_args`
- `cohort`
- `created_utc`
- `source_handoff_ref`
- `warnings`

Scenario labels from the UI preview are mapped before writing the manifest:

- `single-target` -> `single`
- `multi-target` -> `multi`
- `bringup` -> `bringup`

## Status

The status schema is `rt_layout_mc_job_status_v1`.

Lifecycle values:

- `prepared` — after `prepare`; `progress_current` is `0`
- `running` — during `execute` (Monte Carlo subprocess)
- `completed` — subprocess succeeded and post-run validation passed
- `failed` — subprocess non-zero exit or validation failure

Common fields:

- `progress_total`: manifest `run_count`
- `output_paths`: `null` until completed; then `summary_json` and `summary_csv` under `runs/mc/<job_id>.*`
- `command_preview`: rendered `python3 scripts/monte_carlo.py run ...`

On `failed`, status also records `return_code`, `stderr_summary`, and `failure_reason`.

On `completed`, status also records:

- `result_summary_path` — job-local lightweight MC summary (`result_summary.json`)
- `result_link_path` — maintainer handoff index (`result_link.json`)

### Identifier propagation (layout track)

These fields must stay aligned across `manifest.json`, completed `status.json`,
`result_summary.json`, and `result_link.json`:

| Field | Layout namespace | Planning note |
|-------|------------------|---------------|
| `job_id` | MC run label (`--label`) | Maps to `mc_run_label` in pasted Planning ref |
| `geometry_id` | `rt_layout:sha256:*` | **Not** `planning_geometry_id` (`rt_planning:*`) |
| `source_layout_id` | layout lineage hint | May match package `source_layout_id` when aligned |
| `cohort` | evaluation cohort tag | Orthogonal to Planning snapshot ids |

Planning linkage requires maintainer-supplied `linked_package_id` and
`linked_planning_geometry_id` (`planning_snapshot_id` / package ids are not inferred
from layout artifacts).

Maintainer mapping helper (stdout only, no UI import):

```bash
python3 scripts/evaluation/rt_layout_mc_planning_ref.py map \
  runs/rt_sandbox/mc_jobs/<job_id> \
  --linked-package-id 'rt_planning_package:rt_planning_snapshot:sha256:…' \
  --linked-planning-geometry-id 'rt_planning:sha256:…'
```

Golden fixtures: `rt_layout_mc_result_summary_golden_v1.json`,
`rt_layout_mc_result_link_golden_v1.json`, `rt_layout_mc_planning_result_ref_golden_v1.json`.

### Result handoff artifacts

After successful `execute` and validation, the job directory also contains:

- `result_summary.json` (`rt_layout_mc_result_summary_v1`) — manifest identifiers,
  `output_paths`, and lightweight `mc_metrics` copied from `runs/mc/<job_id>.json`
- `result_link.json` (`rt_layout_mc_result_link_v1`) — paths to manifest, status,
  result summary, and MC outputs for maintainer review (not Planning import)

## Boundaries

This execution preparation layer is additive-only:

- no UI execution
- no RT bridge command or protocol change
- no Gazebo launch behavior change
- no replay viewer change
- no tactical, autonomy, tracker, or fusion logic change
- no automatic SA promotion

Monte Carlo jobs can be long-running. Maintainers should review run count,
seed base, geometry freshness, launch args, and cohort labels before running the
rendered command.
