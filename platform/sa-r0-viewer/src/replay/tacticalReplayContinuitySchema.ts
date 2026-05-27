import { z } from "zod";

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

export const rtTacticalReplayContinuitySchema = z.object({
  schema: z.literal("rt_tactical_replay_continuity_v1"),
  source: z.string(),
  continuity_available: z.boolean(),
  capture_candidate_id: z.string(),
  governance_banner: z.string(),
  provenance: z.object({
    imported_from_rt_capture: z.boolean().optional(),
    rt_capture_ref: z.string().optional(),
    tactical_annex_ref: z.string().optional(),
    normalized_manifest_embedded: z.boolean().optional(),
    authority_stopped_at: z.string().optional(),
  }),
  tactical_annex: rtTacticalCaptureAnnexSchema,
});

export type RtTacticalReplayContinuity = z.infer<typeof rtTacticalReplayContinuitySchema>;

export function hasTacticalContinuity(
  block: RtTacticalReplayContinuity | undefined,
): boolean {
  return Boolean(block?.continuity_available && block.tactical_annex);
}
