import { z } from "zod";

const gridSchema = z.object({
  origin_enu_m: z.tuple([z.number(), z.number()]).or(z.array(z.number()).length(2)),
  spacing_m: z.number(),
  size: z.tuple([z.number(), z.number()]).or(z.array(z.number()).length(2)),
});

const countLayerSchema = z.object({
  counts: z.array(z.number()),
  caveat: z.string().optional(),
});

const spatialAggregateSchema = z.object({
  grid: gridSchema,
  layers: z.object({
    ambiguity_density: countLayerSchema.optional(),
    los_degraded: countLayerSchema.optional(),
    first_detection: z
      .object({
        points_enu_m: z.array(z.array(z.number())),
        caveat: z.string().optional(),
      })
      .optional(),
    intercept_outcome: countLayerSchema.optional(),
    topology_sensitivity: countLayerSchema.optional(),
    replay_event_clusters: z
      .object({
        centroids_enu_m: z.array(z.array(z.number())).optional(),
        labels: z.array(z.string()).optional(),
        caveat: z.string().optional(),
      })
      .optional(),
  }),
});

export const replayMcSweepSchema = z.object({
  artifact_type: z.literal("replay_mc_sweep_v1"),
  schema_version: z.literal("replay_mc_sweep_v1"),
  sweep_id: z.string(),
  sweep_kind: z.enum([
    "matched_seed",
    "topology_sweep",
    "sensor_placement_sweep",
    "ingress_variation",
  ]),
  title: z.string(),
  experiment_tags: z.array(z.string()).optional(),
  baseline_topology_key: z.string(),
  governance: z.object({
    notice: z.string(),
    anti_claims: z.array(z.string()).optional(),
  }),
  lineage: z.record(z.unknown()).optional(),
  topology_linkage: z.record(z.unknown()).optional(),
  members: z.array(
    z.object({
      member_id: z.string(),
      pack_id: z.string(),
      demo_bundle_url: z.string(),
      seed: z.number().optional(),
      comparison_hints: z.record(z.unknown()).optional(),
      replay_pattern_tags: z.array(z.string()).optional(),
      replay_pattern_summary: z.string().optional(),
    }),
  ),
  spatial_aggregate: spatialAggregateSchema,
  replay_aggregation: z
    .object({
      outcome_histogram: z.record(z.array(z.number())).optional(),
      dominant_patterns: z.array(z.string()).optional(),
    })
    .optional(),
  replay_narrative_summary: z
    .object({
      headline: z.string(),
      bullets: z.array(z.string()),
      importance_weights: z.record(z.number()).optional(),
    })
    .optional(),
  replay_cohorts: z
    .array(
      z.object({
        cohort_id: z.string(),
        label: z.string(),
        pattern_tags: z.array(z.string()),
        member_indices: z.array(z.number()),
        dominant_summary: z.string(),
        anomaly_member_indices: z.array(z.number()).optional(),
      }),
    )
    .optional(),
  presentation_walkthrough: z
    .object({
      walkthrough_id: z.string(),
      headline: z.string(),
      steps: z.array(
        z.object({
          step_id: z.string(),
          label: z.string(),
          kind: z.enum(["chapter", "cohort_filmstrip", "compare_pair", "analytics_panel"]),
          copy: z.string(),
          member_index: z.number().optional(),
          cohort_id: z.string().optional(),
          filmstrip_indices: z.array(z.number()).optional(),
          chapter_index: z.number().optional(),
        }),
      ),
    })
    .optional(),
});

export type ReplayMcSweep = z.infer<typeof replayMcSweepSchema>;

export const sweepsIndexSchema = z.object({
  artifact_type: z.literal("scenario_sweeps_index_v1"),
  schema_version: z.literal("scenario_sweeps_index_v1"),
  governance: z.object({ notice: z.string() }).optional(),
  sweeps: z.array(
    z.object({
      sweep_id: z.string(),
      title: z.string(),
      sweep_kind: z.string(),
      member_count: z.number(),
      experiment_tags: z.array(z.string()).optional(),
      baseline_topology_key: z.string(),
      sweep_manifest_url: z.string(),
    }),
  ),
});

export type SweepsIndex = z.infer<typeof sweepsIndexSchema>;

export const SWEEPS_INDEX_URL = "/demo/sweeps_index.json";

export const SWEEP_KIND_LABELS: Record<string, string> = {
  matched_seed: "Matched-seed replay family",
  topology_sweep: "Topology sweep",
  sensor_placement_sweep: "Sensor placement sweep",
  ingress_variation: "Ingress variation sweep",
};
