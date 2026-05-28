import type { BackgroundStaleReason } from "@/workstation/backgroundSessionRowCognition";
import { staleReasonLabel } from "@/workstation/backgroundSessionRowCognition";

export function backgroundPullChipLabel(pullAgeLabel: string): string {
  return `last pull: ${pullAgeLabel}`;
}

export function backgroundStaleChipLabels(
  reasons: readonly BackgroundStaleReason[],
): string[] {
  return reasons.map((r) => staleReasonLabel(r));
}

export function backgroundPollPausedLabel(): string {
  return "poll paused";
}

export function backgroundPullingLabel(): string {
  return "pulling";
}

export function backgroundPullFaultLabel(error: string): string {
  return `pull fault: ${error}`;
}
