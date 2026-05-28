import { groupEnrichedRows } from "./advisoryTriageGrouping";
import type {
  AdvisoryExperimentRollup,
  EnrichedAdvisoryRow,
} from "./advisoryTypes";
import { ADVISORY_GOVERNANCE_BANNER } from "./advisoryTypes";

export interface ClientExportPreviewV2 {
  schema: "rt_advisory_batch_review_v2";
  dry_run: true;
  governance_banner: string;
  summary: {
    total: number;
    readiness_cohorts: Partial<Record<string, number>>;
    blocker_groups: Partial<
      Record<string, { count: number; exemplar_capture_ids: string[] }>
    >;
  };
  grouped: {
    by_queue_band: Record<string, string[]>;
    by_blocker_group: Record<string, string[]>;
    by_readiness_cohort: Record<string, string[]>;
  };
  standup: {
    priority_capture_ids: string[];
    top_blocker_groups: Array<{
      group_id: string;
      count: number;
      exemplar_capture_ids: string[];
    }>;
    cohort_counts: Partial<Record<string, number>>;
    warn_only_notes: string[];
    experiment_rollup?: AdvisoryExperimentRollup | null;
  };
  rollups_v2?: {
    readiness_cohorts_v2?: Partial<Record<string, number>>;
    handoff_rollup_by_stage?: Partial<Record<string, number>>;
    stale_age_warn_count?: number;
  };
}

const PRIORITY_BANDS = new Set(["P0_block", "P1_error", "P2_normalize"]);

export function buildClientExportPreviewV2(
  rows: EnrichedAdvisoryRow[],
  experimentRollup?: AdvisoryExperimentRollup | null,
): ClientExportPreviewV2 {
  const readiness_cohorts: Partial<Record<string, number>> = {};
  const blocker_counts: Partial<Record<string, number>> = {};
  const blocker_exemplars: Partial<Record<string, string[]>> = {};

  for (const row of rows) {
    readiness_cohorts[row.readiness_cohort] =
      (readiness_cohorts[row.readiness_cohort] ?? 0) + 1;
    for (const g of row.blocker_groups) {
      blocker_counts[g] = (blocker_counts[g] ?? 0) + 1;
      const list = blocker_exemplars[g] ?? [];
      if (list.length < 5) {
        list.push(row.capture_candidate_id);
        blocker_exemplars[g] = list;
      }
    }
  }

  const blocker_groups: ClientExportPreviewV2["summary"]["blocker_groups"] = {};
  for (const [gid, count] of Object.entries(blocker_counts)) {
    blocker_groups[gid] = {
      count: count ?? 0,
      exemplar_capture_ids: blocker_exemplars[gid] ?? [],
    };
  }

  const by_queue_band: Record<string, string[]> = {};
  const by_blocker_group: Record<string, string[]> = {};
  const by_readiness_cohort: Record<string, string[]> = {};

  for (const g of groupEnrichedRows(rows, "queue_band")) {
    by_queue_band[g.key] = g.rows.map((r) => r.capture_candidate_id);
  }
  for (const g of groupEnrichedRows(rows, "blocker")) {
    by_blocker_group[g.key] = g.rows.map((r) => r.capture_candidate_id);
  }
  for (const g of groupEnrichedRows(rows, "cohort")) {
    by_readiness_cohort[g.key] = g.rows.map((r) => r.capture_candidate_id);
  }

  const priority_capture_ids = rows
    .filter((r) => PRIORITY_BANDS.has(String(r.queue_priority.band)))
    .map((r) => r.capture_candidate_id);

  const top_blocker_groups = Object.entries(blocker_groups)
    .map(([group_id, info]) => ({
      group_id,
      count: info?.count ?? 0,
      exemplar_capture_ids: info?.exemplar_capture_ids ?? [],
    }))
    .sort((a, b) => b.count - a.count)
    .slice(0, 5);

  const warn_only_notes = [
    "experiment eligibility is warn-only; per-capture advisory is authority",
    "client preview — not SA corpus authority",
  ];

  const readiness_cohorts_v2: Partial<Record<string, number>> = {};
  let stale_age_warn_count = 0;
  for (const row of rows) {
    const c2 = row.readiness_cohort_v2 ?? row.readiness_cohort;
    readiness_cohorts_v2[c2] = (readiness_cohorts_v2[c2] ?? 0) + 1;
    if (row.stale_age_hours != null) stale_age_warn_count += 1;
  }
  const handoff_rollup_by_stage: Partial<Record<string, number>> = {};
  for (const row of rows) {
    // Keep mapping stable with advisoryAggregationV2 laneForCohort.
    const map: Record<string, string> = {
      needs_normalize: "normalize",
      needs_review: "review",
      needs_approve: "approve",
      needs_prepare: "prepare",
      ready_for_commit_advisory: "import_advisory",
      stale_review: "review",
      stale_approve: "approve",
    };
    const lane = map[row.readiness_cohort];
    if (lane) {
      handoff_rollup_by_stage[lane] = (handoff_rollup_by_stage[lane] ?? 0) + 1;
    }
  }

  return {
    schema: "rt_advisory_batch_review_v2",
    dry_run: true,
    governance_banner: ADVISORY_GOVERNANCE_BANNER,
    summary: {
      total: rows.length,
      readiness_cohorts,
      blocker_groups,
    },
    grouped: {
      by_queue_band,
      by_blocker_group,
      by_readiness_cohort,
    },
    standup: {
      priority_capture_ids,
      top_blocker_groups,
      cohort_counts: readiness_cohorts,
      warn_only_notes,
      experiment_rollup: experimentRollup ?? undefined,
    },
    rollups_v2: {
      readiness_cohorts_v2,
      handoff_rollup_by_stage,
      stale_age_warn_count,
    },
  };
}

export function formatExportJsonPreview(preview: ClientExportPreviewV2): string {
  return JSON.stringify(preview, null, 2);
}
