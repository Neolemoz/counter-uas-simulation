import { z } from "zod";

export const experimentRunQueueSchema = z.object({
  artifact_type: z.literal("experiment_run_queue_v1"),
  schema_version: z.literal("experiment_run_queue_v1"),
  queue_id: z.string(),
  created_at: z.string(),
  manifest_ref: z.string(),
  manifest_id: z.string(),
  governance_banner: z.string(),
  dry_run: z.boolean().optional(),
  jobs: z.array(
    z.object({
      job_id: z.string(),
      scenario_pack_id: z.string().optional(),
      status: z.enum(["pending", "running", "completed", "failed", "skipped", "dry_run"]),
      phase: z.string().optional(),
      started_at: z.string().optional(),
      finished_at: z.string().optional(),
      artifact_refs: z.array(z.string()).optional(),
      error_hint: z.string().optional(),
      provenance: z
        .object({
          scenario_pack_ref: z.string().nullable().optional(),
          bundle_path: z.string().nullable().optional(),
          corpus_ref: z.string().nullable().optional(),
        })
        .optional(),
    }),
  ),
  steps: z
    .array(
      z.object({
        step_id: z.string(),
        step_type: z.string(),
        status: z.string(),
        duration_ms: z.number().optional(),
        command: z.string().optional(),
        hint: z.string().optional(),
      }),
    )
    .optional(),
});

export type ExperimentRunQueue = z.infer<typeof experimentRunQueueSchema>;

export const validationMirrorSchema = z.object({
  artifact_type: z.literal("experiment_validation_mirror_v1"),
  schema_version: z.literal("experiment_validation_mirror_v1"),
  scenario_pack_id: z.string(),
  scenario_pack_ref: z.string().optional(),
  ok: z.boolean(),
  issues: z.array(z.string()).optional(),
  warnings: z.array(z.string()).optional(),
  checked_at: z.string().optional(),
  governance_banner: z.string().optional(),
});

export type ValidationMirror = z.infer<typeof validationMirrorSchema>;

export const orchestrationIndexSchema = z.object({
  artifact_type: z.literal("orchestration_mirror_index_v1"),
  entries: z.array(
    z.object({
      id: z.string(),
      kind: z.string(),
      url: z.string(),
    }),
  ),
  governance_banner: z.string().optional(),
});
