import { z } from "zod";

const governanceSchema = z.object({
  notice: z.string().optional(),
  anti_claims: z.array(z.string()).optional(),
});

export const federationCorpusGroupSchema = z.object({
  corpus_group_id: z.string(),
  corpus_id: z.string(),
  index_artifact_path: z.string(),
  release_id: z.string().optional(),
  study_label: z.string().optional(),
  orchestration_scope: z.string().optional(),
});

export const replayFederationManifestSchema = z.object({
  artifact_type: z.literal("replay_federation_manifest_v1"),
  schema_version: z.string(),
  federation_id: z.string(),
  generation_revision: z.string(),
  governance: governanceSchema,
  corpus_groups: z.array(federationCorpusGroupSchema),
  parent_federation_ref: z.string().optional(),
  publication_collection_refs: z.array(z.string()).optional(),
  federation_lineage_refs: z.array(z.string()).optional(),
});

export const federationLineageEdgeSchema = z.object({
  edge_id: z.string(),
  from_corpus_group_id: z.string(),
  to_corpus_group_id: z.string(),
  ref_kind: z.string(),
  evidence: z
    .object({
      source_paths: z.array(z.string()).optional(),
      note: z.string().optional(),
    })
    .optional(),
});

export const replayFederationLineageGraphSchema = z.object({
  artifact_type: z.literal("replay_federation_lineage_graph_v1"),
  schema_version: z.string(),
  federation_id: z.string(),
  generation_revision: z.string(),
  governance: governanceSchema.optional(),
  edges: z.array(federationLineageEdgeSchema),
});

export const federationGroupSummarySchema = z.object({
  corpus_group_id: z.string(),
  corpus_id: z.string().optional(),
  release_id: z.string().optional(),
  study_label: z.string().optional(),
  index_artifact_path: z.string(),
  index_revision: z.string().optional(),
  entry_count: z.number(),
  entry_kind_counts: z.record(z.number()).optional(),
});

export const replayFederationIndexSchema = z.object({
  artifact_type: z.literal("replay_federation_index_v1"),
  schema_version: z.string(),
  federation_id: z.string(),
  generation_revision: z.string(),
  governance: governanceSchema,
  manifest_ref: z.string(),
  corpus_group_summaries: z.array(federationGroupSummarySchema),
  federation_revision: z.string(),
  continuity_index_ref: z.string().optional(),
  lineage_graph_ref: z.string().optional(),
});

export const federationIntegrityFindingSchema = z.object({
  kind: z.string(),
  severity: z.string(),
  message: z.string(),
  supersession_note: z.string().optional(),
  groups: z.array(z.string()).optional(),
});

export const replayFederationIntegrityReportSchema = z.object({
  artifact_type: z.literal("replay_federation_integrity_report_v1"),
  schema_version: z.string(),
  federation_id: z.string(),
  generation_revision: z.string(),
  governance: governanceSchema.optional(),
  findings: z.array(federationIntegrityFindingSchema),
  integrity_ok: z.boolean(),
  report_revision: z.string().optional(),
});

export const federationPublicationMemberSchema = z.object({
  corpus_group_id: z.string(),
  artifact_path: z.string(),
  artifact_kind: z.string(),
  sha256: z.string(),
  publication_revision: z.string().optional(),
});

export const replayFederationPublicationCollectionSchema = z.object({
  artifact_type: z.literal("replay_federation_publication_collection_v1"),
  schema_version: z.string(),
  federation_id: z.string(),
  collection_id: z.string(),
  generation_revision: z.string(),
  governance: governanceSchema.optional(),
  members: z.array(federationPublicationMemberSchema),
});

export const replayFederationContinuityIndexSchema = z.object({
  artifact_type: z.literal("replay_federation_continuity_index_v1"),
  schema_version: z.string(),
  federation_id: z.string(),
  generation_revision: z.string(),
  governance: governanceSchema.optional(),
  canonical_release_pairs: z.array(
    z.object({
      entry_id: z.string(),
      canonical_sha256: z.string().optional(),
      release_sha256: z.string().optional(),
      bytes_match: z.boolean().optional(),
    }),
  ),
  shared_entry_count: z.number().optional(),
  publication_chain_head: z
    .object({
      artifact_path: z.string(),
      sha256: z.string(),
      release_id: z.string().optional(),
    })
    .optional(),
});

export const orchestrationFederationRecoveryContinuitySchema = z.object({
  artifact_type: z.literal("orchestration_federation_recovery_continuity_v1"),
  schema_version: z.string(),
  federation_id: z.string(),
  governance: governanceSchema.optional(),
  corpus_group_scopes: z.array(
    z.object({
      corpus_group_id: z.string(),
      study_label: z.string().optional(),
      orchestration_scope: z.string().optional(),
    }),
  ),
  recovery_issues: z.array(
    z.object({
      kind: z.string(),
      severity: z.string(),
      message: z.string(),
      corpus_group_id: z.string().optional(),
    }),
  ),
  continuity_ok: z.boolean(),
  batch_audit_ref: z.string().optional(),
  lineage_index_ref: z.string().optional(),
});

export type ReplayFederationManifest = z.infer<typeof replayFederationManifestSchema>;
export type ReplayFederationIndex = z.infer<typeof replayFederationIndexSchema>;
export type ReplayFederationLineageGraph = z.infer<typeof replayFederationLineageGraphSchema>;
export type ReplayFederationIntegrityReport = z.infer<
  typeof replayFederationIntegrityReportSchema
>;
export type ReplayFederationPublicationCollection = z.infer<
  typeof replayFederationPublicationCollectionSchema
>;
export type ReplayFederationContinuityIndex = z.infer<
  typeof replayFederationContinuityIndexSchema
>;
export type OrchestrationFederationRecoveryContinuity = z.infer<
  typeof orchestrationFederationRecoveryContinuitySchema
>;
