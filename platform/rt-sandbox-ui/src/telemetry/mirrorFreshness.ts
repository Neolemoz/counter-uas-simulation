/** UI-only entity_pose_mirror freshness derivation (Web ↔ Gazebo Step 4). */

import { formatLastPullAge } from "@/telemetry/pullAge";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type { StatusBadgeTone } from "@/workstation/StatusBadge";

/** Mirrors bridge default `GovernanceConfig.telemetry_stale_s`. */
export const TELEMETRY_STALE_DEFAULT_S = 30;

export type MirrorFreshnessState = "fresh" | "stale" | "unavailable";

export type MirrorFreshnessView = {
  state: MirrorFreshnessState;
  label: string;
  tone: StatusBadgeTone;
  detail: string;
  lastMirrorUtc: string | null;
};

function stringOrNull(value: unknown): string | null {
  if (value === null || value === undefined) return null;
  return String(value);
}

function numberOrNull(value: unknown): number | null {
  if (typeof value === "number" && Number.isFinite(value)) return value;
  return null;
}

/** Stale threshold aligned with live poll cadence or bridge telemetry_stale_s. */
export function mirrorStaleThresholdMs(livePollHz: number | null): number {
  if (livePollHz != null && livePollHz > 0) {
    return Math.max(2000, Math.ceil(2000 / livePollHz));
  }
  return TELEMETRY_STALE_DEFAULT_S * 1000;
}

function pickTelemetryHealth(
  mirrorPayload: Record<string, unknown>,
  worldSummary?: Record<string, unknown>,
): string | null {
  const fromMirror = stringOrNull(mirrorPayload.telemetry_health);
  if (fromMirror) return fromMirror;
  return stringOrNull(worldSummary?.telemetry_health);
}

function unavailable(detail: string, lastMirrorUtc: string | null): MirrorFreshnessView {
  return {
    state: "unavailable",
    label: "Mirror: unavailable",
    tone: "error",
    detail,
    lastMirrorUtc,
  };
}

function stale(
  detail: string,
  lastMirrorUtc: string | null,
): MirrorFreshnessView {
  return {
    state: "stale",
    label: "Mirror: stale",
    tone: "warn",
    detail,
    lastMirrorUtc,
  };
}

function fresh(
  detail: string,
  lastMirrorUtc: string | null,
): MirrorFreshnessView {
  return {
    state: "fresh",
    label: "Mirror: fresh",
    tone: "ok",
    detail,
    lastMirrorUtc,
  };
}

export function deriveMirrorFreshness({
  mirrorSnapshot,
  worldSummaryPayload,
  sessionHealthPayload,
  connected = true,
  liveBackgroundPollHz = null,
  nowMs = Date.now(),
}: {
  mirrorSnapshot?: ChannelSnapshot;
  worldSummaryPayload?: Record<string, unknown>;
  sessionHealthPayload?: Record<string, unknown>;
  connected?: boolean;
  liveBackgroundPollHz?: number | null;
  nowMs?: number;
}): MirrorFreshnessView {
  if (!connected) {
    return unavailable("Session not connected — no live mirror.", null);
  }

  if (!mirrorSnapshot) {
    return unavailable(
      "No entity_pose_mirror on last telemetry pull.",
      null,
    );
  }

  const lastMirrorUtc = mirrorSnapshot.timestamp_utc || null;
  const payload = mirrorSnapshot.payload ?? {};
  const telemetryHealth = pickTelemetryHealth(payload, worldSummaryPayload);

  if (telemetryHealth === "feedback_lost") {
    return unavailable(
      "Adapter telemetry feedback lost — mirror not authoritative.",
      lastMirrorUtc,
    );
  }

  const pollHz =
    liveBackgroundPollHz ??
    numberOrNull(sessionHealthPayload?.live_background_poll_hz);
  const thresholdMs = mirrorStaleThresholdMs(pollHz);

  const parsed = lastMirrorUtc ? Date.parse(lastMirrorUtc) : NaN;
  const staleByAge =
    !Number.isFinite(parsed) || nowMs - parsed > thresholdMs;

  if (telemetryHealth === "stale" || staleByAge) {
    const ageText = lastMirrorUtc
      ? formatLastPullAge(lastMirrorUtc, nowMs)
      : "unknown age";
    return stale(
      telemetryHealth === "stale"
        ? `Bridge reports telemetry_health=stale (mirror event ${ageText}).`
        : `Mirror event older than ${Math.round(thresholdMs / 1000)}s (${ageText}).`,
      lastMirrorUtc,
    );
  }

  const source =
    typeof payload.source === "string" ? payload.source : "unknown";
  const ageText = lastMirrorUtc
    ? formatLastPullAge(lastMirrorUtc, nowMs)
    : "just now";
  return fresh(
    `entity_pose_mirror from ${source}; event ${ageText}.`,
    lastMirrorUtc,
  );
}
