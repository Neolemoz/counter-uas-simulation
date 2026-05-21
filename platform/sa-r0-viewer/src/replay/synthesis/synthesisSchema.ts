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

export const corpusRefSchema = z.object({
  corpus_id: z.string(),
  entry_id: z.string(),
  lineage_parent_ids: z.array(z.string()).optional(),
  index_revision: z.string().optional(),
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
  corpus_ref: corpusRefSchema.optional(),
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

export const corpusIndexEntrySchema = z.object({
  entry_id: z.string(),
  corpus_id: z.string(),
  entry_kind: z.string(),
  lineage_parent_ids: z.array(z.string()),
  primary_artifact_path: z.string(),
  generation_tool: z.string().optional(),
  content_revision: z.string().optional(),
  sha256: z.string().optional(),
  navigation_tags: z.array(z.string()).optional(),
  reviewer_category: z.string().optional(),
  replay_family: z.string().optional(),
  chronology_group: z.string().optional(),
  navigation_hint: z.string().optional(),
  release_generation_id: z.string().optional(),
  publication_revision_lineage: z.array(z.string()).optional(),
  evolution_tags: z.array(z.string()).optional(),
  replay_chronology_descriptor: z.string().optional(),
  replay_scope: z.record(z.unknown()).optional(),
  sweep_scope: z.record(z.unknown()).optional(),
  presentation_scope: z.record(z.unknown()).optional(),
  export_scope: z.record(z.unknown()).optional(),
  derived_from: z.array(z.record(z.unknown())).optional(),
});

export const lineageEdgeSchema = z.object({
  edge_id: z.string(),
  parent_entry_id: z.string(),
  child_entry_id: z.string(),
  ref_kind: z.string(),
  evidence: z.record(z.unknown()).optional(),
});

export const replayCorpusIndexSchema = z.object({
  artifact_type: z.literal("replay_corpus_index_v1"),
  schema_version: z.literal("replay_corpus_index_v1"),
  corpus_id: z.string(),
  generation_revision: z.string().optional(),
  index_revision: z.string().optional(),
  governance: z
    .object({
      notice: z.string().optional(),
      anti_claims: z.array(z.string()).optional(),
    })
    .optional(),
  entries: z.array(corpusIndexEntrySchema),
  lineage_edges: z.array(lineageEdgeSchema).optional(),
});

export const driftFindingSchema = z.object({
  finding_id: z.string(),
  kind: z.string(),
  severity: z.enum(["info", "warning", "error"]),
  message: z.string(),
  entry_id: z.string().optional(),
  path: z.string().optional(),
});

export const replayCorpusDriftReportSchema = z.object({
  artifact_type: z.literal("replay_corpus_drift_report_v1"),
  schema_version: z.literal("replay_corpus_drift_report_v1"),
  corpus_id: z.string(),
  index_revision: z.string().optional(),
  governance: z
    .object({
      notice: z.string().optional(),
      anti_claims: z.array(z.string()).optional(),
    })
    .optional(),
  summary: z
    .object({
      total: z.number().optional(),
      by_kind: z.record(z.number()).optional(),
      by_severity: z.record(z.number()).optional(),
    })
    .optional(),
  findings: z.array(driftFindingSchema),
});

export const corpusReleaseManifestSchema = z.object({
  artifact_type: z.literal("replay_corpus_release_manifest_v1"),
  schema_version: z.literal("replay_corpus_release_manifest_v1"),
  release_id: z.string(),
  corpus_id: z.string(),
  indexed_entry_ids: z.array(z.string()),
  parent_release_ids: z.array(z.string()).optional(),
  evolution_chain: z.array(z.record(z.unknown())).optional(),
  publication_revision: z.string().optional(),
  governance: z
    .object({
      notice: z.string().optional(),
      anti_claims: z.array(z.string()).optional(),
    })
    .optional(),
});

export const chronologyTierSchema = z.object({
  tier_id: z.string(),
  descriptor: z.string(),
  release_generation_id: z.string(),
  entry_ids: z.array(z.string()),
  entry_count: z.number(),
});

export const replayCorpusEvolutionManifestSchema = z.object({
  artifact_type: z.literal("replay_corpus_evolution_manifest_v1"),
  schema_version: z.literal("replay_corpus_evolution_manifest_v1"),
  corpus_id: z.string(),
  generation_revision: z.string().optional(),
  governance: z
    .object({
      notice: z.string().optional(),
      anti_claims: z.array(z.string()).optional(),
    })
    .optional(),
  releases: z.array(z.record(z.unknown())),
  chronology_tiers: z.array(chronologyTierSchema),
  cross_release_diffs: z.array(z.record(z.unknown())).optional(),
});

export const replayCorpusEvolutionSummarySchema = z.object({
  artifact_type: z.literal("replay_corpus_evolution_summary_v1"),
  schema_version: z.literal("replay_corpus_evolution_summary_v1"),
  corpus_id: z.string(),
  governance: z
    .object({
      notice: z.string().optional(),
      anti_claims: z.array(z.string()).optional(),
    })
    .optional(),
  topology_sensitivity_evolution: z.record(z.unknown()).optional(),
  ambiguity_trend_rollup: z.record(z.unknown()).optional(),
  pattern_evolution_rollup: z.record(z.unknown()).optional(),
  divergence_chronology: z.array(z.string()).optional(),
  long_horizon_family_narratives: z
    .array(
      z.object({
        replay_family: z.string(),
        summary: z.string(),
        caveat: z.string().optional(),
      }),
    )
    .optional(),
});

export type CorpusRef = z.infer<typeof corpusRefSchema>;
export type CorpusIndexEntry = z.infer<typeof corpusIndexEntrySchema>;
export type ReplayCorpusIndex = z.infer<typeof replayCorpusIndexSchema>;
export type ReplayCorpusDriftReport = z.infer<typeof replayCorpusDriftReportSchema>;
export type CorpusReleaseManifest = z.infer<typeof corpusReleaseManifestSchema>;
export type ReplayCorpusEvolutionManifest = z.infer<typeof replayCorpusEvolutionManifestSchema>;
export type ReplayCorpusEvolutionSummary = z.infer<typeof replayCorpusEvolutionSummarySchema>;
export type CrossSweepSynthesis = z.infer<typeof crossSweepSynthesisSchema>;
export type ReplayLinkageIndex = z.infer<typeof replayLinkageIndexSchema>;
