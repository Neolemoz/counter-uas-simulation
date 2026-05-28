import type { CaptureHandoffRow } from "@/bridge/types";
import { enrichAdvisoryRow } from "./advisoryAggregate";
import type {
  EnrichedAdvisoryRow,
  ExperimentHandoffRollup,
  FilterPresetId,
  HandoffRollup,
  MultiCaptureCohorts,
  ReadinessCohortV2Id,
  SessionAdvisorySummary,
  TemplatePackId,
} from "./advisoryTypes";
import { ADVISORY_GOVERNANCE_BANNER } from "./advisoryTypes";

export const STALE_AGE_HOURS = 24;

export const FILTER_PRESET_OPTIONS: { id: FilterPresetId; label: string }[] = [
  { id: "all_staged", label: "All staged" },
  { id: "blocked_today", label: "Blocked / reject / defer" },
  { id: "defer_queue", label: "Defer queue" },
  { id: "import_advisory_only", label: "Import-advisory only" },
  { id: "review_backlog", label: "Review backlog" },
  { id: "normalize_failures", label: "Normalization failures" },
  { id: "experiment_warn_only", label: "Experiment warn" },
  { id: "lineage_review", label: "Lineage warnings" },
];

export const STANDUP_PASS_LABELS = [
  "Pass A — Firefight",
  "Pass B — Normalize",
  "Pass C — Review",
  "Pass D — Package",
  "Pass E — Import advisory",
];

export interface StandupPassDef {
  pass_id: string;
  label: string;
  preset_ids: FilterPresetId[];
  cohort_hint?: string;
}

/** Mirrors Python STANDUP_PASSES — pass selector filters triage view only. */
export const STANDUP_PASSES: StandupPassDef[] = [
  {
    pass_id: "pass_a_firefight",
    label: "Pass A — Firefight",
    preset_ids: ["blocked_today", "defer_queue"],
  },
  {
    pass_id: "pass_b_normalize",
    label: "Pass B — Normalize",
    preset_ids: ["normalize_failures"],
  },
  {
    pass_id: "pass_c_review",
    label: "Pass C — Review",
    preset_ids: ["review_backlog"],
  },
  {
    pass_id: "pass_d_package",
    label: "Pass D — Package",
    preset_ids: [],
    cohort_hint: "needs_prepare",
  },
  {
    pass_id: "pass_e_import_advisory",
    label: "Pass E — Import advisory",
    preset_ids: ["import_advisory_only"],
  },
];

export function primaryPresetForPass(pass: StandupPassDef): FilterPresetId | null {
  if (pass.preset_ids.length > 0) return pass.preset_ids[0];
  return null;
}

export function countRowsForStandupPass(
  rows: EnrichedAdvisoryRow[],
  pass: StandupPassDef,
): number {
  const preset = primaryPresetForPass(pass);
  let filtered = preset ? applyFilterPreset(rows, preset) : [...rows];
  if (pass.cohort_hint) {
    filtered = filtered.filter((r) => r.readiness_cohort === pass.cohort_hint);
  }
  return filtered.length;
}

function rowMatchesPreset(row: EnrichedAdvisoryRow, presetId: FilterPresetId): boolean {
  if (presetId === "all_staged") return true;
  const { status } = row;
  if (presetId === "blocked_today") {
    return status.blocked || row.blocker_groups.includes("terminal_block");
  }
  if (presetId === "defer_queue") {
    return (status.block_reasons ?? []).includes("handoff_import_deferred");
  }
  if (presetId === "import_advisory_only") {
    return status.advisory_state === "import_ready";
  }
  if (presetId === "review_backlog") {
    return (
      status.advisory_state === "capture_ready" ||
      status.advisory_state === "review_complete"
    );
  }
  if (presetId === "normalize_failures") {
    return row.blocker_groups.includes("normalization");
  }
  if (presetId === "experiment_warn_only") {
    return row.blocker_groups.includes("experiment_warn");
  }
  if (presetId === "lineage_review") {
    return (status.lineage_warnings?.length ?? 0) > 0;
  }
  return true;
}

export function applyFilterPreset(
  rows: EnrichedAdvisoryRow[],
  presetId: FilterPresetId,
): EnrichedAdvisoryRow[] {
  if (presetId === "all_staged") return rows;
  return rows.filter((r) => rowMatchesPreset(r, presetId));
}

export function applyFocusSet(
  rows: EnrichedAdvisoryRow[],
  focusIds: ReadonlySet<string> | null,
): EnrichedAdvisoryRow[] {
  if (!focusIds || focusIds.size === 0) {
    return rows.map((r) => ({ ...r, in_focus_set: undefined }));
  }
  return rows
    .filter((r) => focusIds.has(r.capture_candidate_id))
    .map((r) => ({ ...r, in_focus_set: true }));
}

export function readinessCohortV2(
  row: EnrichedAdvisoryRow,
  experimentWarnIds?: ReadonlySet<string>,
): ReadinessCohortV2Id {
  if (row.in_focus_set) return "focus_highlight";
  if (experimentWarnIds?.has(row.capture_candidate_id)) {
    return "experiment_handoff_warn";
  }
  if (row.blocker_groups.length >= 2) return "multi_blocker";
  if (row.stale_age_hours != null) {
    if (row.readiness_cohort === "needs_review") return "stale_review";
    if (row.readiness_cohort === "needs_approve") return "stale_approve";
  }
  return row.readiness_cohort;
}

export function enrichAdvisoryRowV2(
  row: CaptureHandoffRow,
  sessionLifecycleState?: string | null,
  options?: {
    experimentWarn?: boolean;
    experimentWarnIds?: ReadonlySet<string>;
    generatedAtMs?: number;
    inFocusSet?: boolean;
  },
): EnrichedAdvisoryRow {
  const base = enrichAdvisoryRow(row, sessionLifecycleState, {
    experimentWarn: options?.experimentWarn,
  });
  let stale: number | null = null;
  if (options?.generatedAtMs != null) {
    const ageH = (Date.now() - options.generatedAtMs) / 3600000;
    if (ageH >= STALE_AGE_HOURS) stale = Math.round(ageH * 10) / 10;
  }
  const withFocus: EnrichedAdvisoryRow = {
    ...base,
    stale_age_hours: stale,
    in_focus_set: options?.inFocusSet ? true : undefined,
  };
  return {
    ...withFocus,
    readiness_cohort_v2: readinessCohortV2(withFocus, options?.experimentWarnIds),
  };
}

function laneForCohort(cohort: string): keyof HandoffRollup["by_stage"] | null {
  const map: Record<string, keyof HandoffRollup["by_stage"]> = {
    needs_normalize: "normalize",
    needs_review: "review",
    needs_approve: "approve",
    needs_prepare: "prepare",
    ready_for_commit_advisory: "import_advisory",
    stale_review: "review",
    stale_approve: "approve",
  };
  return map[cohort] ?? null;
}

export function rollupHandoff(rows: EnrichedAdvisoryRow[]): HandoffRollup {
  const by_stage: HandoffRollup["by_stage"] = {};
  let blocked_count = 0;
  let terminal_count = 0;
  for (const row of rows) {
    const cohort = row.readiness_cohort;
    if (cohort === "blocked") {
      blocked_count += 1;
      continue;
    }
    if (cohort === "terminal") {
      terminal_count += 1;
      continue;
    }
    const lane = laneForCohort(cohort);
    if (lane) by_stage[lane] = (by_stage[lane] ?? 0) + 1;
  }
  return { by_stage, blocked_count, terminal_count };
}

export function rollupMultiCaptureCohorts(rows: EnrichedAdvisoryRow[]): MultiCaptureCohorts {
  const by_primary_lane: Record<string, number> = {};
  let stale_age_warn_count = 0;
  for (const row of rows) {
    const lane = laneForCohort(row.readiness_cohort);
    if (lane) by_primary_lane[lane] = (by_primary_lane[lane] ?? 0) + 1;
    if (row.stale_age_hours != null) stale_age_warn_count += 1;
  }
  return {
    by_primary_lane,
    stale_age_warn_count,
    note: "lane counts are advisory cognition only",
  };
}

export function buildSessionAdvisorySummaryV2(
  rows: CaptureHandoffRow[],
  sessionLifecycleState?: string | null,
  options?: { experimentRollup?: ExperimentHandoffRollup | null },
): SessionAdvisorySummary | null {
  if (!rows.length) return null;
  const warnSet = options?.experimentRollup?.warn_capture_ids?.length
    ? new Set(options.experimentRollup.warn_capture_ids)
    : undefined;
  const enriched = rows.map((r) =>
    enrichAdvisoryRowV2(r, sessionLifecycleState, {
      experimentWarn: warnSet?.has(r.capture_candidate_id),
      experimentWarnIds: warnSet,
    }),
  );
  const readiness_cohorts_v2: Partial<Record<ReadinessCohortV2Id, number>> = {};
  const readiness_cohorts: SessionAdvisorySummary["readiness_cohorts"] = {};
  const blocker_groups: SessionAdvisorySummary["blocker_groups"] = {};
  for (const e of enriched) {
    const c2 = e.readiness_cohort_v2 ?? e.readiness_cohort;
    readiness_cohorts_v2[c2] = (readiness_cohorts_v2[c2] ?? 0) + 1;
    readiness_cohorts[e.readiness_cohort] = (readiness_cohorts[e.readiness_cohort] ?? 0) + 1;
    for (const g of e.blocker_groups) {
      blocker_groups[g] = (blocker_groups[g] ?? 0) + 1;
    }
  }
  const experiment_handoff_rollup: ExperimentHandoffRollup | null =
    options?.experimentRollup
      ? {
          ...options.experimentRollup,
          note:
            options.experimentRollup.note ??
            "X2 cohort and packet paths are read-only adjacency; not commit authority",
        }
      : null;
  return {
    total: rows.length,
    readiness_cohorts,
    blocker_groups,
    readiness_cohorts_v2,
    multi_capture_cohorts: rollupMultiCaptureCohorts(enriched),
    handoff_rollup: rollupHandoff(enriched),
    experiment_handoff_rollup,
    schema_version: "f8",
  };
}

export function renderTemplatePackPreview(
  summary: SessionAdvisorySummary,
  packId: TemplatePackId,
): string {
  if (packId === "standup_json_v2") {
    return JSON.stringify(
      {
        governance_banner: ADVISORY_GOVERNANCE_BANNER,
        schema_version: "f8",
        summary: {
          total: summary.total,
          readiness_cohorts_v2: summary.readiness_cohorts_v2,
          multi_capture_cohorts: summary.multi_capture_cohorts,
          handoff_rollup: summary.handoff_rollup,
        },
      },
      null,
      2,
    );
  }
  if (packId === "standup_md_minimal") {
    const imp = summary.handoff_rollup?.by_stage?.import_advisory ?? 0;
    return [
      "# Stand-up (minimal)",
      `> ${ADVISORY_GOVERNANCE_BANNER}`,
      `- **Total:** ${summary.total}`,
      `- **Import-advisory (manual commit):** ${imp}`,
    ].join("\n");
  }
  const lines = [
    "# Stand-up daily",
    `> ${ADVISORY_GOVERNANCE_BANNER}`,
    "> Preset ≠ CLI invocation.",
    "",
    ...STANDUP_PASS_LABELS.map((label) => `- ${label}`),
  ];
  const byStage = summary.handoff_rollup?.by_stage;
  if (byStage) {
    lines.push("", "### Handoff stages");
    for (const [k, v] of Object.entries(byStage)) {
      lines.push(`- ${k}: ${v}`);
    }
  }
  return lines.join("\n");
}
