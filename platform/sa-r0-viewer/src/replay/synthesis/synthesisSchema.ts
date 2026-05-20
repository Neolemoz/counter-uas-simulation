import { z } from "zod";

export const cognitionBulletSchema = z.object({
  bullet_id: z.string(),
  observation: z.string(),
  caveat: z.string(),
  related_sweep_ids: z.array(z.string()).optional(),
  topology_group: z.string().optional(),
});

export const cognitionRollupSchema = z.object({
  artifact_type: z.literal("replay_cognition_rollup_v1"),
  schema_version: z.string(),
  bullets: z.array(cognitionBulletSchema),
  groupings: z.array(z.string()).optional(),
});

export const crossSweepSynthesisSchema = z.object({
  artifact_type: z.literal("cross_sweep_synthesis_v1"),
  schema_version: z.string(),
  governance: z
    .object({
      notice: z.string().optional(),
      anti_claims: z.array(z.string()).optional(),
    })
    .optional(),
  sweep_ids: z.array(z.string()),
  pattern_frequency_rollup: z.record(z.unknown()).optional(),
  ambiguity_concentration_comparison: z.record(z.unknown()).optional(),
  topology_sensitivity_rollup: z.record(z.unknown()).optional(),
  los_instability_rollup: z.record(z.unknown()).optional(),
  divergence_rollup: z.record(z.unknown()).optional(),
  cognition_rollup: cognitionRollupSchema.optional(),
  interpretation_caveats: z.array(z.string()).optional(),
});

export const linkageNodeSchema = z.object({
  sweep_id: z.string(),
  baseline_topology_key: z.string().optional(),
  topology_keys: z.array(z.string()).optional(),
  dominant_patterns: z.array(z.string()).optional(),
  experiment_tags: z.array(z.string()).optional(),
});

export const linkageEdgeSchema = z.object({
  edge_id: z.string(),
  source: z.string(),
  target: z.string(),
  link_kind: z.enum([
    "shared_pattern",
    "shared_topology",
    "metric_similarity",
    "storyline_reference",
  ]),
  evidence: z.record(z.unknown()).optional(),
  copy: z.string(),
});

export const replayLinkageIndexSchema = z.object({
  artifact_type: z.literal("replay_linkage_index_v1"),
  schema_version: z.string(),
  governance: z
    .object({
      notice: z.string().optional(),
    })
    .optional(),
  nodes: z.array(linkageNodeSchema),
  edges: z.array(linkageEdgeSchema),
});

export type CrossSweepSynthesis = z.infer<typeof crossSweepSynthesisSchema>;
export type ReplayLinkageIndex = z.infer<typeof replayLinkageIndexSchema>;
