import { z } from "zod";

export const ANNEX_REVIEW_GOVERNANCE_BANNER =
  "RT ANNEX REVIEW — replay-boundary timelines only; not operational authority";

const timelineEntrySchema = z.record(z.unknown());

export const rtTacticalCaptureAnnexSchema = z.object({
  schema: z.literal("rt_tactical_capture_annex_v1"),
  origin: z.string().optional(),
  capture_candidate_id: z.string().optional(),
  ephemeral_session_ref: z.string().optional(),
  final_tactical_mode: z.string().optional(),
  selected_id: z.string().nullable().optional(),
  assigned_target: z.string().nullable().optional(),
  selected_timeline: z.array(timelineEntrySchema).optional(),
  assignment_timeline: z.array(timelineEntrySchema).optional(),
  tti_timeline: z.array(timelineEntrySchema).optional(),
  recommendation_timeline: z.array(timelineEntrySchema).optional(),
  mode_switches: z.array(timelineEntrySchema).optional(),
  pause_resume_transitions: z.array(timelineEntrySchema).optional(),
  assignment_lock_events: z.array(timelineEntrySchema).optional(),
  target_switch_events: z.array(timelineEntrySchema).optional(),
  authority_label: z.literal("replay_boundary_scoped"),
  governance_banner: z.string(),
});

export type TacticalCaptureAnnex = z.infer<typeof rtTacticalCaptureAnnexSchema>;

export const experimentAnnexBundleSchema = z.object({
  schema: z.literal("rt_experiment_annex_bundle_v1"),
  experiment_id: z.string().optional(),
  entries: z.array(
    z.object({
      run_id: z.string(),
      capture_candidate_id: z.string().nullable().optional(),
      annex: rtTacticalCaptureAnnexSchema,
    }),
  ),
});

export type ExperimentAnnexBundle = z.infer<typeof experimentAnnexBundleSchema>;

export function parseTacticalAnnexJson(text: string): TacticalCaptureAnnex {
  return rtTacticalCaptureAnnexSchema.parse(JSON.parse(text));
}

export function timelineCounts(annex: TacticalCaptureAnnex): Record<string, number> {
  return {
    mode_switches: annex.mode_switches?.length ?? 0,
    assignment_timeline: annex.assignment_timeline?.length ?? 0,
    pause_resume_transitions: annex.pause_resume_transitions?.length ?? 0,
    recommendation_timeline: annex.recommendation_timeline?.length ?? 0,
    selected_timeline: annex.selected_timeline?.length ?? 0,
    tti_timeline: annex.tti_timeline?.length ?? 0,
    assignment_lock_events: annex.assignment_lock_events?.length ?? 0,
    target_switch_events: annex.target_switch_events?.length ?? 0,
  };
}
