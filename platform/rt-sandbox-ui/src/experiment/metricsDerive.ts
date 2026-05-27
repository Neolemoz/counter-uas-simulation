import {
  experimentMetricsReportSchema,
  METRICS_GOVERNANCE_BANNER,
  type ExperimentAnalyticsReport,
  type ExperimentBatchSpec,
  type ExperimentManifest,
  type ExperimentMetricsReport,
  type ExperimentRun,
  type ExperimentSpec,
  type HandoffEligibility,
  type PerRunExtended,
  type RollupExtended,
} from "./experimentSchema";
import { compareBadges, sideFromPinnedRun } from "./experimentCompare";
import { safeParseMetricsReport } from "./experimentImportGuards";
import { cartesianProductSize } from "./experimentSpecCompile";
import type { ComparePairAnalytics, PerRunAnalytics } from "./experimentSchema";

export const FORBIDDEN_ROLLUP_KEYS = new Set([
  "success_rate",
  "winner",
  "best_run",
  "readiness_index",
]);

export type StagingReader = (stagingRef: string) => string | null;

export type DeriveMetricsOptions = {
  batchSpec?: ExperimentBatchSpec;
  spec?: ExperimentSpec;
  stagingReader?: StagingReader;
  maintainerAckPoseReviewed?: boolean;
};

function terrainFromRun(run: ExperimentRun) {
  const snap = run.snapshot?.terrain_context;
  const vis = run.visibility_context;
  return {
    nearest_ridge: snap?.nearest_ridge ?? null,
    elevation_band: snap?.elevation_band ?? null,
    terrain_profile_ref: run.terrain_profile_ref ?? snap?.terrain_profile_ref ?? null,
    f4_layers_enabled: vis?.f4_layer_preset ? [vis.f4_layer_preset] : [],
    los_cognition_label: vis?.los_cognition_label ?? snap?.visibility_hint ?? null,
    occlusion_marker_count: vis?.occlusion_marker_count ?? null,
  };
}

function experimentClassForRun(
  run: ExperimentRun,
  spec?: ExperimentSpec,
): string | null {
  return run.experiment_class ?? spec?.experiment_class ?? null;
}

function specFingerprintForRun(run: ExperimentRun): string | null {
  return run.spec_fingerprint ?? null;
}

function axisSignatureFromCoords(coords: Record<string, string> | null | undefined): string | null {
  if (!coords || !Object.keys(coords).length) return null;
  return Object.keys(coords)
    .sort()
    .map((k) => `${k}=${coords[k]}`)
    .join(";");
}

export function perRunExtendedFromRun(
  run: ExperimentRun,
  f1Row: PerRunAnalytics,
  priorAssigned: string | null,
  spec?: ExperimentSpec,
): PerRunExtended {
  const terrain = terrainFromRun(run);
  const assigned = f1Row.assigned_id_short ?? null;
  const assignDelta =
    priorAssigned != null && assigned != null && priorAssigned !== assigned;

  const annexCounts = run.tactical_annex_summary?.timeline_counts;
  const pauseCount = annexCounts?.pause_resume_transitions ?? null;

  return {
    run_id: run.run_id,
    experiment_class: experimentClassForRun(run, spec),
    spec_fingerprint: specFingerprintForRun(run),
    matrix_coords: run.matrix_coords ?? null,
    axis_signature: axisSignatureFromCoords(run.matrix_coords),
    repeat_group_id: run.repeat_group_id ?? null,
    repeat_index: run.repeat_index ?? null,
    terrain_profile_ref: terrain.terrain_profile_ref ?? null,
    nearest_ridge: terrain.nearest_ridge,
    elevation_band: terrain.elevation_band,
    f4_layers_enabled: terrain.f4_layers_enabled,
    los_cognition_label: terrain.los_cognition_label,
    occlusion_marker_count: terrain.occlusion_marker_count,
    mode_at_capture: f1Row.tactical_mode ?? null,
    assign_delta_from_prior: assignDelta,
    autonomous_pause_count: pauseCount,
    handoff_eligibility_hint: f1Row.has_capture ? "unknown" : "ineligible",
  };
}

function countMapInc(map: Record<string, number>, key: string | null | undefined): void {
  const k = key ?? "unknown";
  map[k] = (map[k] ?? 0) + 1;
}

export function rollupExtended(
  perRun: PerRunExtended[],
  f1Report: ExperimentAnalyticsReport,
  spec?: ExperimentSpec,
): RollupExtended {
  const counts_by_class: Record<string, number> = {};
  const ridge_counts: Record<string, number> = {};
  const band_counts: Record<string, number> = {};
  const los_label_counts: Record<string, number> = {};
  const annex_event_totals: Record<string, number> = {};

  let assign_change_count = 0;
  let tti_present_count = 0;

  const fingerprintGroups = new Map<
    string,
    { run_count: number; capture_count: number; normalization_status_counts: Record<string, number> }
  >();

  for (const row of perRun) {
    countMapInc(counts_by_class, row.experiment_class);
    countMapInc(ridge_counts, row.nearest_ridge);
    countMapInc(band_counts, row.elevation_band);
    countMapInc(los_label_counts, row.los_cognition_label);
    if (row.assign_delta_from_prior) assign_change_count += 1;

    const f1 = f1Report.per_run.find((r) => r.run_id === row.run_id);
    if (f1?.tti_s != null) tti_present_count += 1;

    const fp = row.spec_fingerprint ?? "unknown";
    if (!fingerprintGroups.has(fp)) {
      fingerprintGroups.set(fp, {
        run_count: 0,
        capture_count: 0,
        normalization_status_counts: {},
      });
    }
    const g = fingerprintGroups.get(fp)!;
    g.run_count += 1;
    if (f1?.has_capture) g.capture_count += 1;
    const norm = f1?.normalization_status_ref ?? "unavailable";
    g.normalization_status_counts[norm] = (g.normalization_status_counts[norm] ?? 0) + 1;
  }

  for (const row of f1Report.per_run) {
    const counts = row.annex_timeline_counts;
    if (!counts) continue;
    for (const [k, v] of Object.entries(counts)) {
      if (typeof v === "number") {
        annex_event_totals[k] = (annex_event_totals[k] ?? 0) + v;
      }
    }
  }

  let expected_cells = 0;
  if (spec?.compile_strategy === "cartesian" && spec.matrix_axes?.length) {
    expected_cells = cartesianProductSize(spec.matrix_axes);
  } else if (spec?.manifest_expectation?.min_run_count) {
    expected_cells = spec.manifest_expectation.min_run_count;
  }

  const populated_cells = perRun.filter((r) => r.matrix_coords != null).length;
  const missing_cells = Math.max(0, expected_cells - populated_cells);

  const rollup: RollupExtended = {
    class_rollup: { counts_by_class },
    terrain_rollup: { ridge_counts, band_counts },
    visibility_rollup: { los_label_counts },
    tactical_rollup: {
      mode_counts: { ...f1Report.rollup.mode_counts },
      assign_change_count,
      tti_present_count,
      annex_event_totals,
    },
    repeatability_rollup: {
      fingerprints: [...fingerprintGroups.entries()]
        .sort(([a], [b]) => a.localeCompare(b))
        .map(([spec_fingerprint, g]) => ({
          spec_fingerprint,
          run_count: g.run_count,
          capture_count: g.capture_count,
          normalization_status_counts: g.normalization_status_counts,
        })),
    },
    matrix_rollup: {
      expected_cells,
      populated_cells,
      missing_cells,
    },
  };

  for (const key of Object.keys(rollup.tactical_rollup.mode_counts)) {
    if (FORBIDDEN_ROLLUP_KEYS.has(key)) {
      throw new Error(`forbidden rollup key: ${key}`);
    }
  }

  return rollup;
}

function matrixAxisDiffCount(
  a: Record<string, string> | null,
  b: Record<string, string> | null,
): number {
  if (!a || !b) return -1;
  const keys = new Set([...Object.keys(a), ...Object.keys(b)]);
  let diffs = 0;
  for (const k of keys) {
    if ((a[k] ?? "") !== (b[k] ?? "")) diffs += 1;
  }
  return diffs;
}

function annexTotalCount(row: PerRunExtended, f1: ExperimentAnalyticsReport): number {
  const f1Row = f1.per_run.find((r) => r.run_id === row.run_id);
  const counts = f1Row?.annex_timeline_counts;
  if (!counts) return 0;
  return Object.values(counts).reduce((s, v) => s + (typeof v === "number" ? v : 0), 0);
}

export function buildExtendedComparePairs(
  manifest: ExperimentManifest,
  perRun: PerRunExtended[],
  f1Report: ExperimentAnalyticsReport,
): ComparePairAnalytics[] {
  const runs = [...manifest.runs].sort((a, b) => a.run_id.localeCompare(b.run_id));
  const pairs: ComparePairAnalytics[] = [];

  for (let i = 0; i < runs.length; i += 1) {
    for (let j = i + 1; j < runs.length; j += 1) {
      const a = runs[i];
      const b = runs[j];
      const extA = perRun.find((r) => r.run_id === a.run_id);
      const extB = perRun.find((r) => r.run_id === b.run_id);
      if (!extA || !extB) continue;

      const badges = compareBadges(sideFromPinnedRun(a), sideFromPinnedRun(b));

      if (extA.experiment_class !== extB.experiment_class) {
        badges.push({
          id: "class_mismatch",
          label: "class_mismatch",
          detail: `${extA.experiment_class} vs ${extB.experiment_class}`,
        });
      }

      const f1a = f1Report.per_run.find((r) => r.run_id === a.run_id);
      const f1b = f1Report.per_run.find((r) => r.run_id === b.run_id);
      if (Boolean(f1a?.has_capture) !== Boolean(f1b?.has_capture)) {
        badges.push({
          id: "capture_asymmetric",
          label: "capture_asymmetric",
          detail: "capture presence differs",
        });
      }

      if (
        extA.nearest_ridge !== extB.nearest_ridge ||
        extA.elevation_band !== extB.elevation_band
      ) {
        badges.push({
          id: "terrain_context_diff",
          label: "terrain_context_diff",
        });
      }

      if (extA.los_cognition_label !== extB.los_cognition_label) {
        badges.push({
          id: "visibility_label_diff",
          label: "visibility_label_diff",
        });
      }

      const annexA = annexTotalCount(extA, f1Report);
      const annexB = annexTotalCount(extB, f1Report);
      if (annexA !== annexB) {
        badges.push({
          id: "annex_count_delta",
          label: "annex_count_delta",
          detail: `${annexA} vs ${annexB}`,
        });
      }

      const axisDiffs = matrixAxisDiffCount(extA.matrix_coords, extB.matrix_coords);
      if (axisDiffs === 1) {
        badges.push({ id: "matrix_axis_diff", label: "matrix_axis_diff" });
      }

      pairs.push({
        run_id_a: a.run_id,
        run_id_b: b.run_id,
        badges,
      });
    }
  }

  return pairs.sort(
    (x, y) => x.run_id_a.localeCompare(y.run_id_a) || x.run_id_b.localeCompare(y.run_id_b),
  );
}

export function evaluateHandoffEligibility(
  manifest: ExperimentManifest,
  f1Report: ExperimentAnalyticsReport,
  options?: DeriveMetricsOptions,
): HandoffEligibility {
  const gates: HandoffEligibility["gates"] = [];
  const per_run_gates: HandoffEligibility["per_run_gates"] = [];

  const captureMissing = f1Report.per_run.filter((r) => !r.has_capture).length;
  gates.push({
    id: "all_captures_present",
    pass: captureMissing === 0,
    detail:
      captureMissing === 0
        ? "all runs have capture"
        : `${captureMissing}/${f1Report.per_run.length} runs missing capture`,
  });

  let normFail = false;
  const reader = options?.stagingReader;
  for (const row of f1Report.per_run) {
    let status = row.normalization_status_ref;
    if (reader && row.capture_staging_ref) {
      status = reader(row.capture_staging_ref) ?? "unavailable";
    }
    if (status !== "normalized") normFail = true;
  }
  gates.push({
    id: "normalization_available",
    pass: !normFail && f1Report.per_run.every((r) => r.has_capture),
    detail: normFail ? "normalization not normalized for all captures" : "ok",
  });

  gates.push({
    id: "lifecycle_importable",
    pass: true,
    detail: "lifecycle not evaluated in P0 derive",
  });

  gates.push({
    id: "pose_cognition_ack",
    pass: Boolean(options?.maintainerAckPoseReviewed),
    detail: options?.maintainerAckPoseReviewed ? "acknowledged" : "not acknowledged",
  });

  const classes = new Set(
    manifest.runs.map((r) => r.experiment_class).filter(Boolean) as string[],
  );
  gates.push({
    id: "no_class_mismatch",
    pass: classes.size <= 1,
    detail: classes.size <= 1 ? "single class" : `classes: ${[...classes].join(", ")}`,
  });

  const allPass = gates.every((g) => g.pass);
  let experiment_level: HandoffEligibility["experiment_level"] = allPass
    ? "eligible"
    : "ineligible";

  const eligibleRuns = f1Report.per_run.filter((r) => r.has_capture).length;
  if (!allPass && eligibleRuns > 0 && eligibleRuns < f1Report.per_run.length) {
    experiment_level = "partial";
  }

  for (const row of f1Report.per_run) {
    const runGates = [
      {
        id: "has_capture",
        pass: row.has_capture,
        detail: row.has_capture ? "capture present" : "no capture",
      },
    ];
    per_run_gates.push({
      run_id: row.run_id,
      eligible: runGates.every((g) => g.pass),
      gates: runGates,
    });
  }

  return { experiment_level, gates, per_run_gates };
}

export function deriveExperimentMetrics(
  manifest: ExperimentManifest,
  f1Report: ExperimentAnalyticsReport,
  options?: DeriveMetricsOptions,
): ExperimentMetricsReport {
  const sorted = [...manifest.runs].sort((a, b) => a.run_id.localeCompare(b.run_id));
  const per_run_extended: PerRunExtended[] = [];
  let priorFp: string | null = null;
  let priorAssigned: string | null = null;

  for (const run of sorted) {
    const f1Row = f1Report.per_run.find((r) => r.run_id === run.run_id);
    if (!f1Row) {
      throw new Error(`missing f1 per_run for ${run.run_id}`);
    }
    const fp = specFingerprintForRun(run);
    const prior =
      fp && fp === priorFp ? priorAssigned : null;
    const ext = perRunExtendedFromRun(run, f1Row, prior, options?.spec);
    per_run_extended.push(ext);
    priorFp = fp;
    priorAssigned = f1Row.assigned_id_short ?? null;
  }

  const classes = manifest.runs.map((r) => r.experiment_class).filter(Boolean);
  const experiment_class =
    classes[0] ?? options?.spec?.experiment_class ?? "unknown";

  const report: ExperimentMetricsReport = {
    schema: "rt_experiment_metrics_report_v1",
    experiment_id: manifest.experiment_id,
    experiment_class,
    governance_banner: METRICS_GOVERNANCE_BANNER,
    spec_fingerprint: per_run_extended[0]?.spec_fingerprint ?? null,
    per_run_extended,
    compare_pairs_extended: buildExtendedComparePairs(manifest, per_run_extended, f1Report),
    rollup_extended: rollupExtended(per_run_extended, f1Report, options?.spec),
    handoff_eligibility: evaluateHandoffEligibility(manifest, f1Report, options),
  };

  return experimentMetricsReportSchema.parse(report);
}

export function exportMetricsJson(
  report: ExperimentMetricsReport,
  opts?: { derived_at_utc?: string },
): string {
  const body = { ...report };
  if (opts?.derived_at_utc) {
    body.derived_at_utc = opts.derived_at_utc;
  }
  return JSON.stringify(experimentMetricsReportSchema.parse(body), null, 2);
}

export function parseMetricsJson(text: string): ExperimentMetricsReport {
  const parsed = safeParseMetricsReport(text);
  if (!parsed.ok) throw new Error(parsed.error);
  return parsed.data;
}
