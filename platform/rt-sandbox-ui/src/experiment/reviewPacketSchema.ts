import { z } from "zod";

export const REVIEW_PACKET_GOVERNANCE_BANNER =
  "RT EXPERIMENT REVIEW PACKET — advisory export only; not SA import authority";

const artifactRefSchema = z.object({
  kind: z.enum([
    "f1_analytics",
    "f5_metrics",
    "f5b_fidelity",
    "f3_annex_cache",
    "cohort_index",
  ]),
  path: z.string(),
  sha256: z.string().optional(),
});

export const reviewPacketSchema = z
  .object({
    schema: z.literal("rt_experiment_review_packet_v1"),
    packet_id: z.string(),
    created_at_utc: z.string(),
    governance_banner: z.string(),
    scope: z.object({
      cohort_id: z.string().nullable().optional(),
      primary_manifest_ref: z.string().nullable().optional(),
      secondary_manifest_ref: z.string().nullable().optional(),
    }),
    artifact_refs: z.array(artifactRefSchema).optional(),
    compare_mode: z.string().optional(),
    compare_run_ids: z.array(z.string()).optional(),
    review_steps_completed: z.array(z.string()).optional(),
  })
  .strict();

export type ExperimentReviewPacket = z.infer<typeof reviewPacketSchema>;
