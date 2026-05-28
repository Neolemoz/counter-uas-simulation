import type { CompareModeId } from "./experimentUnifiedReview";
import type { ExperimentManifest } from "./experimentSchema";
import {
  REVIEW_PACKET_GOVERNANCE_BANNER,
  reviewPacketSchema,
  type ExperimentReviewPacket,
} from "./reviewPacketSchema";
export type ReportDockPresence = {
  f1_analytics: boolean;
  f3_annex: boolean;
  f5_metrics: boolean;
  f5b_fidelity: boolean;
};
import type { WorkbenchV2State } from "./workbenchV2State";

function utcNow(): string {
  return new Date().toISOString().replace(/\.\d{3}Z$/, "Z");
}

export function buildReviewPacketPreview(options: {
  v2State: WorkbenchV2State;
  manifest: ExperimentManifest;
  presence: ReportDockPresence;
  compareRunIds?: string[];
}): ExperimentReviewPacket {
  const { v2State, manifest, presence, compareRunIds } = options;
  const artifact_refs: ExperimentReviewPacket["artifact_refs"] = [];
  if (presence.f1_analytics) {
    artifact_refs.push({
      kind: "f1_analytics",
      path: `runs/rt_sandbox/experiments/${manifest.experiment_id}/reports/analytics.json`,
    });
  }
  if (presence.f5_metrics) {
    artifact_refs.push({
      kind: "f5_metrics",
      path: `runs/rt_sandbox/experiments/${manifest.experiment_id}/reports/metrics.json`,
    });
  }
  if (presence.f5b_fidelity) {
    artifact_refs.push({
      kind: "f5b_fidelity",
      path: `runs/rt_sandbox/experiments/${manifest.experiment_id}/reports/fidelity_metrics.json`,
    });
  }
  if (presence.f3_annex) {
    artifact_refs.push({
      kind: "f3_annex_cache",
      path: `runs/rt_sandbox/experiments/${manifest.experiment_id}/annex_cache.json`,
    });
  }
  if (v2State.active_cohort_id) {
    artifact_refs.push({
      kind: "cohort_index",
      path: `cohort://${v2State.active_cohort_id}`,
    });
  }

  const runIds =
    compareRunIds ??
    [v2State.compare_run_a, v2State.compare_run_b].filter(
      (id): id is string => typeof id === "string" && id.length > 0,
    );

  const packet: ExperimentReviewPacket = {
    schema: "rt_experiment_review_packet_v1",
    packet_id: `review-${manifest.experiment_id}-${Date.now()}`,
    created_at_utc: utcNow(),
    governance_banner: REVIEW_PACKET_GOVERNANCE_BANNER,
    scope: {
      cohort_id: v2State.active_cohort_id,
      primary_manifest_ref: v2State.primary_manifest_ref,
      secondary_manifest_ref: v2State.secondary_manifest_ref,
    },
    artifact_refs,
    compare_mode: v2State.compare_mode as CompareModeId,
    compare_run_ids: runIds,
    review_steps_completed: [...v2State.steps_completed],
  };

  return reviewPacketSchema.parse(packet);
}

export function exportReviewPacketJson(packet: ExperimentReviewPacket): string {
  return JSON.stringify(reviewPacketSchema.parse(packet), null, 2);
}
