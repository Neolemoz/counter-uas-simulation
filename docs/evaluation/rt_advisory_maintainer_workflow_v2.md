# RT Advisory Maintainer Workflow v2 (`rt_advisory_maintainer_workflow_v2`)

**Phase:** PLAN-RT-F8 — maintainer ergonomics expansion (docs only)  
**Prerequisite:** [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md) (F7 frozen); PLAT-RT-F7 P0–P2 frozen  
**Authority:** [rt_manual_sa_import_workflow_v1.md](rt_manual_sa_import_workflow_v1.md); [rt_advisory_contamination_gates_v2.md](rt_advisory_contamination_gates_v2.md)

Normative **maintainer ergonomics v2** atop F7 queue bands, blocker groups, and batch export — filter presets, review-lane templates, focus sets, and X2-adjacent cognition. Does **not** replace SA1 write paths or change F7 P0–P7 band ranks.

---

## 1. Core invariants

```text
filter_preset ≠ CLI invocation
review_lane_template ≠ approve authority
rollup_handoff ≠ import_ready
focus_set ≠ cross-session authority merge
experiment_cohort ≠ readiness_cohort_v2
```

Maintainer CLIs remain authority for review, approve, prepare, and commit. F8 presets, lanes, and templates are **cognition aids only**.

---

## 2. Queue ergonomics — filter presets

### 2.1 Purpose

When N captures are staged, maintainers need **named, repeatable filters** for daily stand-up beyond ad-hoc CLI flags. Presets apply sort/filter only — they do not invoke `next_cli` or change advisory ladder rungs.

### 2.2 Normative preset catalog

| Preset ID | Label | Filter semantics | Default sort |
|-----------|-------|------------------|--------------|
| `blocked_today` | Blocked / reject / defer | `blocked: true` OR `terminal_block` group | `queue` ascending |
| `defer_queue` | Deferred imports | `handoff_import_deferred` in `block_reasons` | `queue` ascending |
| `import_advisory_only` | Import-advisory rows | `advisory_state: import_ready` (advisory only) | `queue` ascending |
| `review_backlog` | Review queue | `capture_ready` OR `review_complete` | `queue` ascending |
| `normalize_failures` | Normalization lane | primary group `normalization` | `queue` ascending |
| `experiment_warn_only` | Experiment advisory warns | `experiment_warn` in `blocker_groups` | `queue` ascending |
| `lineage_review` | Lineage warnings present | `lineage_warnings` non-empty | `queue` ascending |
| `all_staged` | Full staging scan | no row filter (scan scope only) | `queue` ascending |

### 2.3 Saved sort profiles

| Profile ID | Behavior |
|------------|----------|
| `queue_default` | `queue_priority.rank` ascending (F7 default) |
| `queue_band_then_id` | band ascending, then `capture_candidate_id` |
| `oldest_staging` | `generated_at` ascending |
| `blocker_group_count` | primary group frequency (report footer order) |

PLAT reference flags: `--preset <id>`, `--sort-profile <id>` — not required in PLAN wave.

### 2.4 Focus set semantics

A **focus set** is an explicit list of `capture_candidate_id` values (from `--focus-captures` or ids file) used to:

- Narrow report/triage UI rows
- Highlight rows in stand-up template output
- **Not** merge advisory state across captures or sessions

| Rule | Requirement |
|------|-------------|
| Per-capture authority | Each ID retains independent ladder state |
| Multi-session | IDs from different sessions may coexist — no merged queue |
| Empty focus | Preset applies to full scan scope |

---

## 3. Review workflow optimization

### 3.1 Maintainer stand-up lanes (time-boxed passes)

Overlay on F7 triage lanes ([rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md) §4):

| Pass | Duration (advisory) | Preset / lane | Outcome |
|------|---------------------|---------------|---------|
| **Pass A — Firefight** | First | `blocked_today`, `defer_queue` | Clear P0_block / terminal_block |
| **Pass B — Normalize** | Second | `normalize_failures` | Unblock P2_normalize band |
| **Pass C — Review** | Third | `review_backlog` | Move P3_review → P4_approve |
| **Pass D — Package** | Fourth | `needs_prepare` cohort (v2) | Advisory packaging queue |
| **Pass E — Import advisory** | Last | `import_advisory_only` | Manual commit gate only |

Passes are **maintainer discipline** — not automated scheduling.

### 3.2 Pairing with F7 P2 stand-up export

F7 `rt_advisory_batch_review_v2` `standup` section maps to F8 passes:

| F7 `standup` field | F8 pass |
|--------------------|---------|
| `priority_capture_ids` | Pass A + B top ranks |
| `top_blocker_groups` | Pass B/C grouping |
| `cohort_counts` | Pass C–E lane occupancy |
| `warn_only_notes` | Experiment + lineage (no commit signal) |

F8 template packs (§5) may render these fields into Markdown/JSON shells.

### 3.3 Review lane templates

Read-only **templates** describing maintainer intent per lane — not executable workflows:

| Template ID | Lane | Maintainer checklist (summary) |
|-------------|------|--------------------------------|
| `lane_normalize` | Normalize | Validate staging → `rt_capture_normalize` → re-derive |
| `lane_review` | Review | `rt_handoff_review.py` ready → reviewed |
| `lane_approve` | Approve | `rt_capture_approve.py` per capture |
| `lane_prepare` | Prepare | `rt_sa_import prepare` / pipeline steps |
| `lane_import_advisory` | Commit gate | Lineage lint → explicit `commit --corpus-dest` |

Templates **must not** embed `--execute` or chained CLI strings that PLAT could auto-run.

---

## 4. Deeper cohort and experiment rollups (cognition)

### 4.1 Readiness cohort v2 (additive buckets)

Extends F7 cohort IDs ([rt_advisory_aggregation_v1.md](rt_advisory_aggregation_v1.md) §4) — see [rt_advisory_aggregation_v2.md](rt_advisory_aggregation_v2.md) for schema.

| Cohort v2 ID | Purpose |
|--------------|---------|
| `stale_review` | `needs_review` + staging age &gt; advisory stale threshold (warn) |
| `stale_approve` | `needs_approve` + age warn |
| `multi_blocker` | ≥2 primary blocker groups |
| `experiment_handoff_warn` | F5 + X2 adjacency warn-only |

Forbidden: `readiness_score`, `operational_ready`.

### 4.2 X2 manifest pin + handoff cross-reference

When batch scan includes experiment manifest ref:

- **May read** X2 cohort index entry for `manifest_ref` ([rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md))
- **May read** pinned compare/review packet paths from workbench (paths only — not invoke export)
- **Must not** treat cohort `status` or packet file as commit permission

Rollup fields are defined in aggregation v2 `experiment_handoff_rollup`.

---

## 5. Maintainership ergonomics — template packs

### 5.1 Stand-up template packs

Normative shell types for PLAT P2 — render-only:

| Pack ID | Format | Sections |
|---------|--------|----------|
| `standup_json_v2` | JSON | summary v2 + grouped indexes + warn notes |
| `standup_md_daily` | Markdown | Pass A–E checklist + top 5 priority IDs + blocker table |
| `standup_md_minimal` | Markdown | `total`, top 3 groups, `import_advisory_only` count |

Rules:

- Output files are maintainer opt-in (`--output`); not default pipeline
- Enclosing document `dry_run: true` when emitted from batch helper
- Banner string required in Markdown header

### 5.2 CLI vocabulary (future PLAT)

| Flag | Semantics |
|------|-----------|
| `--preset <id>` | Apply §2.2 filter catalog |
| `--focus-captures <id>[,id...]` | §2.4 focus set |
| `--sort-profile <id>` | §2.3 sort profile |
| `--template-pack <id>` | §5.1 render shell (P2) |
| `--schema v2` | Emit `rt_advisory_batch_summary_v2` (P0) |

Default `report` remains F7-compatible unless `--schema v2` specified.

### 5.3 Copy and banner rules

| Surface | Required copy |
|---------|---------------|
| All bulk outputs | F6 governance banner |
| Cohort v2 chips | "Advisory cohort — not operational readiness" |
| Import-advisory preset | "Manual commit required — `rt_sa_import commit`" |
| X2 adjacency strip | "Experiment cohort ≠ handoff authority" |

---

## 6. Bulk workflow v2 sequence

Extends F7 §5.1:

1. **`scan`** — unchanged
2. **`report`** — optional `--preset`, `--focus-captures`, `--schema v2`
3. **`standup-export` / template pack** — render-only (P2)
4. **Preview** — corpus-preview refinement read-only (P2)
5. **Act (manual)** — SA1 CLIs per capture — never chained auto-commit

### 6.1 Defaults and guards

| Rule | Requirement |
|------|-------------|
| Preset ≠ execute | No subprocess from preset application |
| Focus set ≠ merge | Per-capture ladder unchanged |
| Template ≠ pipeline | No embedded auto CLI |
| No `--commit-all` | Forbidden |

---

## 7. Explicit non-goals

- Changing F7 P0–P7 queue band rank ranges
- Replacing `rt_handoff_batch_advisory.py` scan semantics
- SA viewer panels or live rollup subscription
- Auto-import on any preset or cohort v2 label
- Distributed or multi-bridge batch execution

---

## 8. Related

- [rt_advisory_aggregation_v2.md](rt_advisory_aggregation_v2.md)
- [rt_advisory_contamination_gates_v2.md](rt_advisory_contamination_gates_v2.md)
- [rt_advisory_maintainer_workflow_v1.md](rt_advisory_maintainer_workflow_v1.md)
- [rt_experiment_cohort_v1.md](rt_experiment_cohort_v1.md)
- [rt_rt_sa_bridge_handoff_v1.md](rt_rt_sa_bridge_handoff_v1.md)
