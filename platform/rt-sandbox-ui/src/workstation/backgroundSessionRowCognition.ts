import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import { cognitionSummary, type HealthBadge } from "@/telemetry/cognition";
import { sessionStateFromSnapshots } from "@/telemetry/channelIndex";
import { formatLastPullAge, isPullAgeStale } from "@/telemetry/pullAge";

export type BackgroundStaleReason = "pull_fault" | "pull_age" | "telemetry";

export type BackgroundSessionRowCognition = {
  lifecycleLabel: string;
  lifecycleTimestampUtc: string | null;
  healthBadges: HealthBadge[];
  staleReasons: BackgroundStaleReason[];
  pullAgeLabel: string;
  isEditingLock: boolean;
};

export function deriveBackgroundSessionRow(
  slot: SessionSlot,
  editingSessionId: string | null,
  nowMs: number,
): BackgroundSessionRowCognition {
  const lifecyclePayload = slot.snapshots.lifecycle_state?.payload;
  const lifecycleLabel = sessionStateFromSnapshots(slot.snapshots);
  const lifecycleTimestampUtc =
    lifecyclePayload && typeof lifecyclePayload.timestamp_utc === "string"
      ? lifecyclePayload.timestamp_utc
      : slot.snapshots.lifecycle_state?.timestamp_utc ?? null;

  const healthPayload = slot.snapshots.session_health?.payload;
  const health = healthPayload
    ? cognitionSummary(healthPayload as Record<string, unknown>)
    : null;

  const staleReasons: BackgroundStaleReason[] = [];
  if (slot.lastError != null) staleReasons.push("pull_fault");
  if (isPullAgeStale("background", slot.lastPullUtc, 1, nowMs)) {
    staleReasons.push("pull_age");
  }
  if (health?.stale) staleReasons.push("telemetry");

  return {
    lifecycleLabel,
    lifecycleTimestampUtc,
    healthBadges: (health?.badges ?? []).slice(0, 2),
    staleReasons,
    pullAgeLabel: formatLastPullAge(slot.lastPullUtc, nowMs),
    isEditingLock: editingSessionId === slot.sessionId,
  };
}

export function staleReasonLabel(reason: BackgroundStaleReason): string {
  switch (reason) {
    case "pull_fault":
      return "stale: pull fault";
    case "pull_age":
      return "stale: pull age";
    case "telemetry":
      return "stale: telemetry";
  }
}
