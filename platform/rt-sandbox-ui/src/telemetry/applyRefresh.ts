import type { BridgeCommandResponse } from "@/bridge/types";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type { TelemetryChannel } from "@/telemetry/constants";

export function worldSummarySnapshotFromApplyResponse(
  _sessionId: string,
  worldSummary: Record<string, unknown>,
): ChannelSnapshot {
  return {
    channel: "world_summary",
    payload: worldSummary,
    timestamp_utc: new Date().toISOString(),
    governance_banner:
      typeof worldSummary.governance_banner === "string"
        ? worldSummary.governance_banner
        : undefined,
  };
}

export function extractWorldSummaryFromApplyResult(
  result: BridgeCommandResponse,
): Record<string, unknown> | null {
  const direct = result.world_summary;
  if (direct && typeof direct === "object" && !Array.isArray(direct)) {
    return direct as Record<string, unknown>;
  }
  return null;
}

export function patchSnapshotsWithApplyWorldSummary(
  snapshots: Partial<Record<TelemetryChannel, ChannelSnapshot>>,
  sessionId: string,
  worldSummary: Record<string, unknown>,
): Partial<Record<TelemetryChannel, ChannelSnapshot>> {
  return {
    ...snapshots,
    world_summary: worldSummarySnapshotFromApplyResponse(sessionId, worldSummary),
  };
}

const REFRESH_ATTEMPTS = 3;
const REFRESH_DELAY_MS = 60;

function worldSummaryReconciled(worldSummary: Record<string, unknown> | undefined): boolean {
  if (!worldSummary) return false;
  const poll = worldSummary.last_poll_utc;
  const cmd = worldSummary.last_command_utc;
  if (typeof poll !== "string" || typeof cmd !== "string") {
    return typeof poll === "string";
  }
  return poll >= cmd;
}

/** Active pull after apply — no passive poll wait; drains entity_pose_mirror + world_summary. */
export async function refreshSessionAfterApply(input: {
  pull: () => Promise<void>;
  readWorldSummary: () => Record<string, unknown> | undefined;
  readEntityMirror: () => ChannelSnapshot | undefined;
}): Promise<void> {
  for (let attempt = 0; attempt < REFRESH_ATTEMPTS; attempt += 1) {
    await input.pull();
    const worldSummary = input.readWorldSummary();
    const mirror = input.readEntityMirror();
    const mirrorEntities = mirror?.payload?.entities;
    const hasMirror =
      Array.isArray(mirrorEntities) && mirrorEntities.length > 0;
    const emptyWorld = Number(worldSummary?.entity_count ?? 0) === 0;
    if (worldSummaryReconciled(worldSummary) && (hasMirror || emptyWorld)) {
      return;
    }
    if (attempt < REFRESH_ATTEMPTS - 1) {
      await new Promise((resolve) => setTimeout(resolve, REFRESH_DELAY_MS));
    }
  }
}
