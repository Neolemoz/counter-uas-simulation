import { z } from "zod";

export const COHORT_GOVERNANCE_BANNER =
  "RT EXPERIMENT COHORT — index references only; per-manifest authority unchanged";

const FORBIDDEN_MANIFEST_REF_KEYS = [
  "sa_corpus_ref",
  "import_ready",
  "readiness_score",
  "winner_run_id",
] as const;

export const manifestRefEntrySchema = z
  .object({
    manifest_ref: z.string().min(1),
    experiment_id: z.string().min(1),
    label: z.string().min(1),
    run_count_hint: z.number().int().nonnegative().optional(),
    spec_fingerprint: z.string().optional(),
    experiment_class: z.string().optional(),
  })
  .strict();

export const experimentCohortIndexSchema = z
  .object({
    schema: z.literal("rt_experiment_cohort_index_v1"),
    cohort_id: z.string().min(1),
    label: z.string().min(1),
    governance_banner: z.string().min(1),
    manifest_refs: z.array(manifestRefEntrySchema).min(1),
    created_at_utc: z.string().optional(),
    tags: z.array(z.string()).optional(),
    notes: z.string().optional(),
  })
  .strict();

export type CohortManifestRef = z.infer<typeof manifestRefEntrySchema>;
export type ExperimentCohortIndex = z.infer<typeof experimentCohortIndexSchema>;

export function assertNoForbiddenManifestRefKeys(raw: unknown): void {
  if (!raw || typeof raw !== "object") return;
  const refs = (raw as { manifest_refs?: unknown }).manifest_refs;
  if (!Array.isArray(refs)) return;
  for (const entry of refs) {
    if (!entry || typeof entry !== "object") continue;
    for (const key of FORBIDDEN_MANIFEST_REF_KEYS) {
      if (key in (entry as Record<string, unknown>)) {
        throw new Error(`forbidden manifest_ref field: ${key}`);
      }
    }
  }
}
