import { z } from "zod";

const promotionLineageEventSchema = z.object({
  event_id: z.string(),
  from_status: z.string(),
  to_status: z.string(),
  actor: z.string(),
  notes: z.string().optional(),
  recorded_at: z.string().optional(),
  pack_fingerprint: z.string().optional(),
  validation_mirror_ok: z.boolean().nullable().optional(),
});

export const scenarioAuthoringManifestSchema = z.object({
  artifact_type: z.literal("scenario_authoring_manifest_v1"),
  schema_version: z.literal("1"),
  pack_id: z.string(),
  parent_pack_id: z.string().nullable().optional(),
  topology_variant_class: z.string().optional(),
  promotion_status: z.enum([
    "draft",
    "linted",
    "validated",
    "promoted",
    "orchestration_ready",
    "deprecated",
    "archived",
  ]),
  validation_snapshot_ref: z.string().optional(),
  validation_pack_fingerprint: z.string().optional(),
  authoring_notes: z.string().optional(),
  promotion_lineage: z.array(promotionLineageEventSchema).optional(),
  orchestration_handoff_refs: z
    .array(
      z.object({
        manifest_id: z.string().optional(),
        job_id: z.string().optional(),
        queue_mirror_id: z.string().optional(),
      }),
    )
    .optional(),
  catalog_entry_id: z.string().optional(),
  updated_at: z.string().optional(),
  promotion_summary_ref: z.string().optional(),
  integrity_summary_ref: z.string().optional(),
  governance: z
    .object({
      notice: z.string(),
      anti_claims: z.array(z.string()).optional(),
    })
    .optional(),
});

export type ScenarioAuthoringManifest = z.infer<typeof scenarioAuthoringManifestSchema>;

export const authoringMirrorIndexSchema = z.object({
  artifact_type: z.literal("scenario_authoring_mirror_index_v1"),
  schema_version: z.literal("1"),
  governance_banner: z.string().optional(),
  entries: z.array(z.object({ pack_id: z.string(), url: z.string() })),
});

export const authoringIntegrityReportSchema = z.object({
  artifact_type: z.literal("scenario_authoring_integrity_report_v1"),
  schema_version: z.literal("1"),
  checked_at: z.string().optional(),
  strict: z.boolean().optional(),
  ok: z.boolean(),
  pack_count: z.number().optional(),
  manifest_count: z.number().optional(),
  errors: z.array(z.string()).optional(),
  warnings: z.array(z.string()).optional(),
  per_pack: z
    .record(
      z.object({
        pack_id: z.string(),
        errors: z.array(z.string()).optional(),
        warnings: z.array(z.string()).optional(),
      }),
    )
    .optional(),
  governance_banner: z.string().optional(),
});

export type AuthoringIntegrityReport = z.infer<typeof authoringIntegrityReportSchema>;

export const AUTHORING_INDEX_URL = "/demo/authoring/index.json";
export const AUTHORING_INTEGRITY_URL = "/demo/authoring/integrity_report.json";

export function authoringManifestUrl(packId: string): string {
  return `/demo/authoring/${packId}.json`;
}

export const LADDER_STATUSES = [
  "draft",
  "linted",
  "validated",
  "promoted",
  "orchestration_ready",
] as const;

export const RETIRED_STATUSES = ["deprecated", "archived"] as const;
