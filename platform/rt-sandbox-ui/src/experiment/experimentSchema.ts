import { z } from "zod";

export const EXPERIMENT_GOVERNANCE_BANNER =
  "RT EXPERIMENT — explanatory compare only; not operational authority";

export const terrainContextSchema = z.object({
  layers_enabled: z.boolean().optional(),
  nearest_ridge: z.string().nullable().optional(),
  elevation_band: z.string().nullable().optional(),
  contour_layers_on: z.boolean().optional(),
  visibility_hint: z.string().optional(),
  terrain_profile_ref: z.string().optional(),
  note: z.string().optional(),
});

export const visibilityContextSchema = z.object({
  f4_layer_preset: z.string().optional(),
  los_cognition_label: z.string().nullable().optional(),
  occlusion_marker_count: z.number().nullable().optional(),
});

export type VisibilityContext = z.infer<typeof visibilityContextSchema>;

export const runSnapshotSchema = z.object({
  tactical_state: z.record(z.unknown()).optional(),
  world_summary: z.record(z.unknown()).optional(),
  lifecycle_state: z.record(z.unknown()).optional(),
  entity_pose_mirror: z.record(z.unknown()).optional(),
  terrain_context: terrainContextSchema.optional(),
});

export const tacticalAnnexSummarySchema = z.object({
  final_tactical_mode: z.string().nullable().optional(),
  selected_id: z.string().nullable().optional(),
  assigned_target: z.string().nullable().optional(),
  timeline_counts: z
    .object({
      mode_switches: z.number().optional(),
      assignment_timeline: z.number().optional(),
      pause_resume_transitions: z.number().optional(),
      recommendation_timeline: z.number().optional(),
    })
    .optional(),
});

export const fidelityContextSchema = z.object({
  enable_fidelity_coupling: z.boolean(),
  adapter_mode: z.string().optional(),
  truth_snapshot_ref: z.string().optional(),
});

export type FidelityContext = z.infer<typeof fidelityContextSchema>;

const f5ManifestSupplementSchema = z.object({
  experiment_class: z.string().optional(),
  spec_fingerprint: z.string().optional(),
  matrix_coords: z.record(z.string()).nullable().optional(),
  repeat_index: z.number().optional(),
  repeat_group_id: z.string().optional(),
  terrain_profile_ref: z.string().optional(),
  visibility_context: visibilityContextSchema.optional(),
  fidelity_context: fidelityContextSchema.optional(),
});

export const experimentRunSchema = z
  .object({
    run_id: z.string(),
    label: z.string(),
    session_id: z.string(),
    recorded_at_utc: z.string(),
    capture_candidate_id: z.string().nullable().optional(),
    capture_staging_ref: z.string().nullable().optional(),
    snapshot: runSnapshotSchema,
    tactical_annex_summary: tacticalAnnexSummarySchema.nullable().optional(),
  })
  .merge(f5ManifestSupplementSchema);

export const experimentManifestSchema = z.object({
  schema: z.literal("rt_experiment_manifest_v1"),
  experiment_id: z.string(),
  created_at_utc: z.string(),
  governance_banner: z.string(),
  runs: z.array(experimentRunSchema),
});

export type ExperimentManifest = z.infer<typeof experimentManifestSchema>;
export type ExperimentRun = z.infer<typeof experimentRunSchema>;
export type RunSnapshot = z.infer<typeof runSnapshotSchema>;
export type TacticalAnnexSummary = z.infer<typeof tacticalAnnexSummarySchema>;

const f5BatchRunMetaSchema = z.object({
  experiment_class: z.string().optional(),
  spec_fingerprint: z.string().optional(),
  matrix_coords: z.record(z.string()).nullable().optional(),
  repeat_index: z.number().optional(),
  repeat_group_id: z.string().optional(),
  terrain_profile_ref: z.string().optional(),
  f4_layer_preset: z.string().optional(),
});

export const batchRunSpecSchema = z
  .object({
    run_id: z.string(),
    label: z.string(),
    template_id: z.string().nullable().optional(),
    dwell_s: z.number().optional(),
    tactical_mode_hint: z.string().optional(),
  })
  .merge(f5BatchRunMetaSchema);

export const experimentBatchSpecSchema = z.object({
  schema: z.literal("rt_experiment_batch_v1"),
  experiment_id: z.string(),
  default_dwell_s: z.number().optional(),
  runs: z.array(batchRunSpecSchema),
});

export type ExperimentBatchSpec = z.infer<typeof experimentBatchSpecSchema>;

export const ANALYTICS_GOVERNANCE_BANNER =
  "RT ANALYTICS — derived summaries only; not operational authority";

export const annexTimelineCountsSchema = z.object({
  mode_switches: z.number().optional(),
  assignment_timeline: z.number().optional(),
  pause_resume_transitions: z.number().optional(),
  recommendation_timeline: z.number().optional(),
});

export const perRunAnalyticsSchema = z.object({
  run_id: z.string(),
  label: z.string(),
  session_id_short: z.string(),
  recorded_at_utc: z.string(),
  dwell_s: z.number().nullable().optional(),
  template_id: z.string().nullable().optional(),
  tactical_mode_hint: z.string().nullable().optional(),
  entity_count: z.number().nullable().optional(),
  adapter_mode: z.string().nullable().optional(),
  sync_health: z.string().nullable().optional(),
  lifecycle_state: z.unknown().nullable().optional(),
  tactical_mode: z.string().nullable().optional(),
  selected_id_short: z.string().nullable().optional(),
  assigned_id_short: z.string().nullable().optional(),
  tti_s: z.number().nullable().optional(),
  autonomous_loop_status: z.string().nullable().optional(),
  has_capture: z.boolean(),
  capture_candidate_id: z.string().nullable().optional(),
  capture_staging_ref: z.string().nullable().optional(),
  normalization_status_ref: z.string(),
  annex_timeline_counts: annexTimelineCountsSchema.nullable().optional(),
  annex_final_mode: z.string().nullable().optional(),
  terrain_nearest_ridge: z.string().nullable().optional(),
});

export const compareBadgeSchema = z.object({
  id: z.string(),
  label: z.string(),
  detail: z.string().optional(),
});

export const comparePairSchema = z.object({
  run_id_a: z.string(),
  run_id_b: z.string(),
  badges: z.array(compareBadgeSchema),
});

export const rollupSchema = z.object({
  run_count: z.number(),
  capture_count: z.number(),
  mode_counts: z.record(z.number()),
  template_ids_used: z.array(z.string()),
});

export const experimentAnalyticsReportSchema = z.object({
  schema: z.literal("rt_experiment_analytics_report_v1"),
  experiment_id: z.string(),
  derived_at_utc: z.string().optional(),
  governance_banner: z.string(),
  per_run: z.array(perRunAnalyticsSchema),
  compare_pairs: z.array(comparePairSchema),
  rollup: rollupSchema,
});

export type PerRunAnalytics = z.infer<typeof perRunAnalyticsSchema>;
export type ComparePairAnalytics = z.infer<typeof comparePairSchema>;
export type ExperimentAnalyticsReport = z.infer<typeof experimentAnalyticsReportSchema>;

export const sweepEntrySchema = z.object({
  entry_id: z.string(),
  label: z.string(),
  template_id: z.string(),
  dwell_s: z.number().optional(),
  tactical_mode_hint: z.string().optional(),
  run_id_suffix: z.string().optional(),
});

export const sweepGroupSchema = z.object({
  group_id: z.string(),
  label: z.string(),
  description: z.string(),
  governance_banner: z.string().optional(),
  compile_strategy: z.enum(["explicit_list", "cartesian"]),
  default_dwell_s: z.number().optional(),
  sweep_entries: z.array(sweepEntrySchema),
});

export const experimentSweepCatalogSchema = z.object({
  schema: z.literal("rt_experiment_sweep_catalog_v1"),
  catalog_id: z.string(),
  governance_banner: z.string(),
  sweep_groups: z.array(sweepGroupSchema),
});

export type SweepEntry = z.infer<typeof sweepEntrySchema>;
export type SweepGroup = z.infer<typeof sweepGroupSchema>;
export type ExperimentSweepCatalog = z.infer<typeof experimentSweepCatalogSchema>;

export const EXPERIMENT_SPEC_GOVERNANCE_BANNER =
  "RT EXPERIMENT — local maintainer planning only; not operational authority";

export const METRICS_GOVERNANCE_BANNER =
  "RT EXPERIMENT METRICS — derived summaries only; not operational authority";

export const experimentClassSchema = z.enum([
  "terrain_comparison",
  "sensor_range_comparison",
  "tactical_mode_comparison",
  "repeatability_sweep",
  "parameter_matrix",
]);

export type ExperimentClass = z.infer<typeof experimentClassSchema>;

export const compileStrategySchema = z.enum([
  "explicit_list",
  "cartesian",
  "repeat_expand",
]);

export const specEntrySchema = z.object({
  entry_id: z.string(),
  label: z.string(),
  template_id: z.string(),
  dwell_s: z.number().optional(),
  tactical_mode_hint: z.string().optional(),
  terrain_profile_ref: z.string().optional(),
  f4_layer_preset: z.string().optional(),
});

export const matrixAxisSchema = z.object({
  axis_id: z.string(),
  values: z.array(z.union([z.string(), z.number()])),
});

export const repeatConfigSchema = z.object({
  repeat_group_id: z.string(),
  count: z.number().int().min(2),
  jitter_s: z.number(),
  base_entry: z.object({
    template_id: z.string(),
    label: z.string(),
    dwell_s: z.number().optional(),
  }),
});

export const experimentSpecSchema = z.object({
  schema: z.literal("rt_experiment_spec_v1"),
  experiment_id: z.string(),
  experiment_class: experimentClassSchema,
  governance_banner: z.string(),
  hypothesis_label: z.string().optional(),
  compile_strategy: compileStrategySchema,
  spec_entries: z.array(specEntrySchema).optional(),
  matrix_axes: z.array(matrixAxisSchema).optional(),
  repeat_config: repeatConfigSchema.optional(),
  default_dwell_s: z.number().optional(),
  manifest_expectation: z
    .object({
      min_run_count: z.number().optional(),
      required_snapshot_channels: z.array(z.string()).optional(),
    })
    .optional(),
  compile_to_batch_path: z.string().optional(),
});

export type ExperimentSpec = z.infer<typeof experimentSpecSchema>;
export type SpecEntry = z.infer<typeof specEntrySchema>;

export const perRunExtendedSchema = z.object({
  run_id: z.string(),
  experiment_class: z.string().nullable(),
  spec_fingerprint: z.string().nullable(),
  matrix_coords: z.record(z.string()).nullable(),
  axis_signature: z.string().nullable(),
  repeat_group_id: z.string().nullable(),
  repeat_index: z.number().nullable(),
  terrain_profile_ref: z.string().nullable(),
  nearest_ridge: z.string().nullable(),
  elevation_band: z.string().nullable(),
  f4_layers_enabled: z.array(z.string()),
  los_cognition_label: z.string().nullable(),
  occlusion_marker_count: z.number().nullable(),
  mode_at_capture: z.string().nullable(),
  assign_delta_from_prior: z.boolean(),
  autonomous_pause_count: z.number().nullable(),
  handoff_eligibility_hint: z.enum(["eligible", "ineligible", "unknown"]),
});

export type PerRunExtended = z.infer<typeof perRunExtendedSchema>;

export const rollupExtendedSchema = z.object({
  class_rollup: z.object({ counts_by_class: z.record(z.number()) }),
  terrain_rollup: z.object({
    ridge_counts: z.record(z.number()),
    band_counts: z.record(z.number()),
  }),
  visibility_rollup: z.object({ los_label_counts: z.record(z.number()) }),
  tactical_rollup: z.object({
    mode_counts: z.record(z.number()),
    assign_change_count: z.number(),
    tti_present_count: z.number(),
    annex_event_totals: z.record(z.number()),
  }),
  repeatability_rollup: z.object({
    fingerprints: z.array(
      z.object({
        spec_fingerprint: z.string(),
        run_count: z.number(),
        capture_count: z.number(),
        normalization_status_counts: z.record(z.number()),
      }),
    ),
  }),
  matrix_rollup: z.object({
    expected_cells: z.number(),
    populated_cells: z.number(),
    missing_cells: z.number(),
  }),
});

export type RollupExtended = z.infer<typeof rollupExtendedSchema>;

export const handoffGateSchema = z.object({
  id: z.string(),
  pass: z.boolean(),
  detail: z.string(),
});

export const handoffEligibilitySchema = z.object({
  experiment_level: z.enum(["eligible", "ineligible", "partial"]),
  gates: z.array(handoffGateSchema),
  per_run_gates: z.array(
    z.object({
      run_id: z.string(),
      eligible: z.boolean(),
      gates: z.array(handoffGateSchema),
    }),
  ),
});

export type HandoffEligibility = z.infer<typeof handoffEligibilitySchema>;

export const experimentMetricsReportSchema = z.object({
  schema: z.literal("rt_experiment_metrics_report_v1"),
  experiment_id: z.string(),
  experiment_class: z.string(),
  governance_banner: z.string(),
  spec_fingerprint: z.string().nullable(),
  derived_at_utc: z.string().optional(),
  per_run_extended: z.array(perRunExtendedSchema),
  compare_pairs_extended: z.array(comparePairSchema),
  rollup_extended: rollupExtendedSchema,
  handoff_eligibility: handoffEligibilitySchema,
});

export type ExperimentMetricsReport = z.infer<typeof experimentMetricsReportSchema>;

export const FIDELITY_METRICS_GOVERNANCE_BANNER =
  "RT EXPERIMENT FIDELITY METRICS — truth-attested summaries are sim-scoped; not SA replay or operational sensor authority";

export const perRunFidelitySchema = z.object({
  run_id: z.string(),
  fidelity_attestation_status: z.enum(["available", "stale", "unavailable"]),
  los_truth_label: z.string().nullable(),
  los_cognition_label: z.string().nullable(),
  visibility_truth_ref: z.string().nullable(),
  dome_truth_ref: z.string().nullable(),
  pose_truth_drift_m: z.number().nullable(),
  agl_truth_m: z.number().nullable(),
  cognition_truth_divergence: z.boolean(),
});

export type PerRunFidelity = z.infer<typeof perRunFidelitySchema>;

export const comparePairFidelitySchema = comparePairSchema;

export type ComparePairFidelity = z.infer<typeof comparePairFidelitySchema>;

export const rollupFidelitySchema = z.object({
  attestation_rollup: z.object({ status_counts: z.record(z.number()) }),
  divergence_rollup: z.object({ cognition_truth_divergence_count: z.number() }),
  repeatability_truth_rollup: z.object({
    truth_fingerprints: z.array(
      z.object({
        spec_fingerprint: z.string(),
        truth_fingerprint: z.string(),
        run_count: z.number(),
        coupling_flag: z.boolean(),
      }),
    ),
  }),
});

export type RollupFidelity = z.infer<typeof rollupFidelitySchema>;

export const experimentFidelityMetricsReportSchema = z.object({
  schema: z.literal("rt_experiment_fidelity_metrics_report_v1"),
  experiment_id: z.string(),
  governance_banner: z.string(),
  spec_fingerprint: z.string().nullable(),
  coupling_required: z.boolean(),
  derived_at_utc: z.string().optional(),
  per_run_fidelity: z.array(perRunFidelitySchema),
  compare_pairs_fidelity: z.array(comparePairFidelitySchema),
  rollup_fidelity: rollupFidelitySchema,
});

export type ExperimentFidelityMetricsReport = z.infer<
  typeof experimentFidelityMetricsReportSchema
>;
