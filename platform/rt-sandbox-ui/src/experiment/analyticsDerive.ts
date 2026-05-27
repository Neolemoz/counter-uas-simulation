import {
  ANALYTICS_GOVERNANCE_BANNER,
  experimentAnalyticsReportSchema,
  type ExperimentAnalyticsReport,
  type ExperimentBatchSpec,
  type ExperimentManifest,
  type ExperimentRun,
  type PerRunAnalytics,
} from "./experimentSchema";
import { compareBadges, sideFromPinnedRun } from "./experimentCompare";
import { safeParseAnalyticsReport } from "./experimentImportGuards";
import { shortId } from "./experimentIds";

function batchRow(
  batchSpec: ExperimentBatchSpec | undefined,
  runId: string,
): ExperimentBatchSpec["runs"][number] | undefined {
  return batchSpec?.runs.find((r) => r.run_id === runId);
}

function tacticalRecord(run: ExperimentRun): Record<string, unknown> {
  const raw = run.snapshot?.tactical_state;
  return raw && typeof raw === "object" ? (raw as Record<string, unknown>) : {};
}

function worldRecord(run: ExperimentRun): Record<string, unknown> {
  const raw = run.snapshot?.world_summary;
  return raw && typeof raw === "object" ? (raw as Record<string, unknown>) : {};
}

export function perRunFromManifest(
  run: ExperimentRun,
  batchSpec?: ExperimentBatchSpec,
): PerRunAnalytics {
  const batch = batchRow(batchSpec, run.run_id);
  const tactical = tacticalRecord(run);
  const world = worldRecord(run);
  const entityCount = world.entity_count;
  const tti = tactical.tti_s;

  return {
    run_id: run.run_id,
    label: run.label,
    session_id_short: shortId(run.session_id) ?? "—",
    recorded_at_utc: run.recorded_at_utc,
    dwell_s: batch?.dwell_s ?? batchSpec?.default_dwell_s ?? null,
    template_id: batch?.template_id ?? null,
    tactical_mode_hint: batch?.tactical_mode_hint ?? null,
    entity_count: typeof entityCount === "number" ? entityCount : null,
    adapter_mode: typeof world.adapter_mode === "string" ? world.adapter_mode : null,
    sync_health: typeof world.sync_health === "string" ? world.sync_health : null,
    lifecycle_state: run.snapshot?.lifecycle_state ?? null,
    tactical_mode:
      typeof tactical.tactical_mode === "string" ? tactical.tactical_mode : null,
    selected_id_short: shortId(
      (tactical.selected_target_id ?? tactical.selected_interceptor_id) as string | undefined,
    ),
    assigned_id_short: shortId(
      (tactical.assigned_target_id ?? tactical.assigned_interceptor_id) as string | undefined,
    ),
    tti_s: typeof tti === "number" ? tti : null,
    autonomous_loop_status:
      typeof tactical.autonomous_loop_status === "string"
        ? tactical.autonomous_loop_status
        : null,
    has_capture: Boolean(run.capture_candidate_id),
    capture_candidate_id: run.capture_candidate_id ?? null,
    capture_staging_ref: run.capture_staging_ref ?? null,
    normalization_status_ref: "unavailable",
    annex_timeline_counts: run.tactical_annex_summary?.timeline_counts ?? null,
    annex_final_mode: run.tactical_annex_summary?.final_tactical_mode ?? null,
    terrain_nearest_ridge: run.snapshot?.terrain_context?.nearest_ridge ?? null,
  };
}

export function rollupFromPerRun(perRun: PerRunAnalytics[]): ExperimentAnalyticsReport["rollup"] {
  const mode_counts: Record<string, number> = {};
  const templateSet = new Set<string>();
  let capture_count = 0;
  for (const row of perRun) {
    if (row.has_capture) capture_count += 1;
    const mode = row.tactical_mode ?? "unknown";
    mode_counts[mode] = (mode_counts[mode] ?? 0) + 1;
    if (row.template_id) templateSet.add(row.template_id);
  }
  return {
    run_count: perRun.length,
    capture_count,
    mode_counts,
    template_ids_used: [...templateSet].sort(),
  };
}

export function buildComparePairs(
  manifest: ExperimentManifest,
  pairKeys?: Array<[string, string]>,
): ExperimentAnalyticsReport["compare_pairs"] {
  const runs = [...manifest.runs].sort((a, b) => a.run_id.localeCompare(b.run_id));
  const pairs: ExperimentAnalyticsReport["compare_pairs"] = [];

  const iterate = (a: ExperimentRun, b: ExperimentRun) => {
    const sideA = sideFromPinnedRun(a);
    const sideB = sideFromPinnedRun(b);
    pairs.push({
      run_id_a: a.run_id,
      run_id_b: b.run_id,
      badges: compareBadges(sideA, sideB),
    });
  };

  if (pairKeys?.length) {
    for (const [idA, idB] of pairKeys) {
      const a = runs.find((r) => r.run_id === idA);
      const b = runs.find((r) => r.run_id === idB);
      if (a && b) iterate(a, b);
    }
    return pairs.sort(
      (x, y) => x.run_id_a.localeCompare(y.run_id_a) || x.run_id_b.localeCompare(y.run_id_b),
    );
  }

  for (let i = 0; i < runs.length; i += 1) {
    for (let j = i + 1; j < runs.length; j += 1) {
      iterate(runs[i], runs[j]);
    }
  }
  return pairs;
}

export function deriveExperimentAnalytics(
  manifest: ExperimentManifest,
  batchSpec?: ExperimentBatchSpec,
): ExperimentAnalyticsReport {
  const sorted = [...manifest.runs].sort((a, b) => a.run_id.localeCompare(b.run_id));
  const per_run = sorted.map((r) => perRunFromManifest(r, batchSpec));
  const report: ExperimentAnalyticsReport = {
    schema: "rt_experiment_analytics_report_v1",
    experiment_id: manifest.experiment_id,
    governance_banner: ANALYTICS_GOVERNANCE_BANNER,
    per_run,
    compare_pairs: buildComparePairs(manifest),
    rollup: rollupFromPerRun(per_run),
  };
  return experimentAnalyticsReportSchema.parse(report);
}

export function exportAnalyticsJson(report: ExperimentAnalyticsReport): string {
  return JSON.stringify(experimentAnalyticsReportSchema.parse(report), null, 2);
}

export function parseAnalyticsJson(text: string): ExperimentAnalyticsReport {
  const parsed = safeParseAnalyticsReport(text);
  if (!parsed.ok) {
    throw new Error(parsed.error);
  }
  return parsed.data;
}
