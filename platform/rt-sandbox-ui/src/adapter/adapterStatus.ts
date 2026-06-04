import {
  sessionRuntimeProfileGovernance,
  sessionRuntimeProfileLabel,
  type SessionRuntimeProfile,
} from "@/runtime/sessionRuntimeProfile";
import { LIVE_MAINTAINER_SMOKE_HINT } from "@/runtime/liveSessionUx";
import {
  deriveLiveCommandHealth,
  type LiveCommandHealthView,
} from "@/runtime/liveCommandHealth";
import { adapterModeLabel } from "@/sync/cognition";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import {
  deriveMirrorFreshness,
  type MirrorFreshnessView,
} from "@/telemetry/mirrorFreshness";
import {
  formatLastPullAge,
  isPullAgeStale,
} from "@/telemetry/pullAge";
import type { StatusBadgeTone } from "@/workstation/StatusBadge";

/** Read-only UI profile — not bridge runtime_kind. */
export type RuntimeProfile = "stub" | "mock_adapter" | "live_adapter";

export const ADAPTER_VISIBILITY_GOVERNANCE =
  "Read-only telemetry mirror — not command authority. Adapter controls are maintainer-only.";

export type FreshnessView = {
  label: string;
  tone: StatusBadgeTone;
  detail?: string;
};

export type AdapterStatusView = {
  profile: RuntimeProfile;
  profileTitle: string;
  profileDescription: string;
  requestedProfile: SessionRuntimeProfile | null;
  requestedProfileLabel: string | null;
  requestedProfileGovernance: string | null;
  runtimeTypeLabel: string;
  adapterMode: string;
  adapterAlive: boolean | null;
  adapterPid: string | null;
  stubAlive: boolean | null;
  adapterEntityCount: number | null;
  launchHealth: string | null;
  telemetryHealth: string | null;
  syncHealth: string | null;
  telemetryRevision: number | null;
  bridgeLastPollUtc: string | null;
  liveProfileActive: boolean;
  liveBackgroundPollHz: number | null;
  lastLivePollUtc: string | null;
  livePollFreshness: FreshnessView;
  maintainerSmokeHint: string | null;
  mirrorFreshness: MirrorFreshnessView;
  commandHealth: LiveCommandHealthView | null;
  telemetryFreshness: FreshnessView;
  syncFreshness: FreshnessView;
  uiPullFreshness: FreshnessView;
};

const PROFILE_META: Record<
  RuntimeProfile,
  { title: string; description: string; runtimeTypeLabel: string }
> = {
  stub: {
    title: "Stub runtime",
    description: "Bridge RuntimeStub — no Gazebo adapter worker.",
    runtimeTypeLabel: "Stub",
  },
  mock_adapter: {
    title: "Mock adapter",
    description: "Adapter worker in mock mode — in-memory sim feedback.",
    runtimeTypeLabel: "Mock adapter",
  },
  live_adapter: {
    title: "Live adapter",
    description: "Adapter worker in live mode — Gazebo/ROS session topics.",
    runtimeTypeLabel: "Live adapter",
  },
};

function asRecord(value: unknown): Record<string, unknown> | undefined {
  return value && typeof value === "object"
    ? (value as Record<string, unknown>)
    : undefined;
}

function boolOrNull(value: unknown): boolean | null {
  if (typeof value === "boolean") return value;
  return null;
}

function stringOrNull(value: unknown): string | null {
  if (value === null || value === undefined) return null;
  return String(value);
}

function numberOrNull(value: unknown): number | null {
  if (typeof value === "number" && Number.isFinite(value)) return value;
  return null;
}

function normalizeMode(
  sessionHealth?: Record<string, unknown>,
  worldSummary?: Record<string, unknown>,
): string {
  const fromWorld = adapterModeLabel(worldSummary);
  if (fromWorld !== "off") return fromWorld;
  const sh = sessionHealth ?? {};
  const nested = asRecord(sh.adapter_health);
  const mode =
    stringOrNull(nested?.mode) ??
    stringOrNull(sh.adapter_mode) ??
    stringOrNull(sh.runtime_mode);
  if (mode) return mode;
  return "off";
}

export function pickAdapterFields(sessionHealth?: Record<string, unknown>): {
  adapterAlive: boolean | null;
  stubAlive: boolean | null;
  adapterMode: string | null;
  adapterPid: string | null;
  adapterEntityCount: number | null;
} {
  const sh = sessionHealth ?? {};
  const nested = asRecord(sh.adapter_health);
  return {
    adapterAlive:
      boolOrNull(nested?.alive) ?? boolOrNull(sh.adapter_alive),
    stubAlive: boolOrNull(sh.stub_alive),
    adapterMode:
      stringOrNull(nested?.mode) ?? stringOrNull(sh.adapter_mode),
    adapterPid: stringOrNull(sh.adapter_pid),
    adapterEntityCount: numberOrNull(sh.adapter_entity_count),
  };
}

export function deriveRuntimeProfile(
  sessionHealth?: Record<string, unknown>,
  worldSummary?: Record<string, unknown>,
): RuntimeProfile {
  const mode = normalizeMode(sessionHealth, worldSummary);
  const { adapterAlive } = pickAdapterFields(sessionHealth);

  if (mode === "live") return "live_adapter";
  if (mode === "mock" || adapterAlive === true) return "mock_adapter";
  return "stub";
}

function healthFreshness(
  health: string | null | undefined,
  kind: "telemetry" | "sync",
): FreshnessView {
  if (!health) {
    return {
      label: kind === "telemetry" ? "Telemetry: unknown" : "Sync: unknown",
      tone: "neutral",
      detail: "No health field on latest pull snapshot.",
    };
  }
  if (health === "ok") {
    return {
      label: kind === "telemetry" ? "Telemetry: fresh" : "Sync: ok",
      tone: "ok",
      detail: `Bridge reports ${kind}_health=ok.`,
    };
  }
  if (health === "stale" || health === "feedback_lost") {
    return {
      label:
        kind === "telemetry"
          ? `Telemetry: ${health}`
          : `Sync: ${health}`,
      tone: health === "feedback_lost" ? "error" : "warn",
      detail: `Bridge reports ${kind}_health=${health}.`,
    };
  }
  return {
    label: `${kind === "telemetry" ? "Telemetry" : "Sync"}: ${health}`,
    tone: health === "mismatch" ? "error" : "warn",
    detail: `Bridge reports ${kind}_health=${health}.`,
  };
}

function mergeHealth(
  primary: string | null | undefined,
  fallback: string | null | undefined,
): string | null {
  if (typeof primary === "string" && primary) return primary;
  if (typeof fallback === "string" && fallback) return fallback;
  return null;
}

function deriveLaunchHealth(
  sessionHealth?: Record<string, unknown>,
): string | null {
  const sh = sessionHealth ?? {};
  const nested = asRecord(sh.adapter_health);
  const alive =
    boolOrNull(nested?.alive) ?? boolOrNull(sh.adapter_alive);
  if (alive === true) return "launched";
  if (alive === false) return "not_running";
  const mode = stringOrNull(nested?.mode) ?? stringOrNull(sh.adapter_mode);
  if (mode === "live" && alive === null) return "unknown";
  return null;
}

function deriveLivePollFreshness(
  lastLivePollUtc: string | null,
  pollHz: number | null,
  nowMs: number,
): FreshnessView {
  if (!pollHz || pollHz <= 0) {
    return {
      label: "Live poll: n/a",
      tone: "neutral",
      detail: "Background poll inactive for this profile.",
    };
  }
  if (!lastLivePollUtc) {
    return {
      label: "Live poll: pending",
      tone: "warn",
      detail: "Waiting for first bridge live background poll.",
    };
  }
  const staleAfterMs = Math.max(2000, Math.ceil((2000 / pollHz)));
  const ageMs = nowMs - Date.parse(lastLivePollUtc);
  const stale = !Number.isFinite(ageMs) || ageMs > staleAfterMs;
  return {
    label: stale ? "Live poll: stale" : "Live poll: fresh",
    tone: stale ? "warn" : "ok",
    detail: `Last bridge live poll ${formatLastPullAge(lastLivePollUtc, nowMs)} (${pollHz} Hz cap).`,
  };
}

function pickLivePollFields(sessionHealth?: Record<string, unknown>): {
  liveBackgroundPollHz: number | null;
  lastLivePollUtc: string | null;
  runtimeProfile: string | null;
} {
  const sh = sessionHealth ?? {};
  return {
    liveBackgroundPollHz: numberOrNull(sh.live_background_poll_hz),
    lastLivePollUtc: stringOrNull(sh.last_live_poll_utc),
    runtimeProfile: stringOrNull(sh.runtime_profile),
  };
}

export function deriveAdapterStatus({
  sessionHealthPayload,
  worldSummaryPayload,
  entityPoseMirrorSnapshot,
  connected = true,
  lastPullUtc,
  pullHz = 1,
  nowMs = Date.now(),
  requestedRuntimeProfile = null,
  editingEnabled = true,
  livePreflightOk = null,
}: {
  sessionHealthPayload?: Record<string, unknown>;
  worldSummaryPayload?: Record<string, unknown>;
  entityPoseMirrorSnapshot?: ChannelSnapshot;
  connected?: boolean;
  lastPullUtc?: string | null;
  pullHz?: number;
  nowMs?: number;
  requestedRuntimeProfile?: SessionRuntimeProfile | null;
  editingEnabled?: boolean;
  livePreflightOk?: boolean | null;
}): AdapterStatusView {
  const sh = sessionHealthPayload ?? {};
  const ws = worldSummaryPayload ?? {};
  const profile = deriveRuntimeProfile(sh, ws);
  const meta = PROFILE_META[profile];
  const adapterFields = pickAdapterFields(sh);
  const adapterMode = normalizeMode(sh, ws);

  const telemetryHealth = mergeHealth(
    typeof ws.telemetry_health === "string" ? ws.telemetry_health : null,
    typeof sh.telemetry_health === "string" ? sh.telemetry_health : null,
  );
  const syncHealth =
    typeof ws.sync_health === "string" ? ws.sync_health : null;

  const bridgeLastPollUtc =
    typeof ws.last_poll_utc === "string"
      ? ws.last_poll_utc
      : typeof sh.last_poll_utc === "string"
        ? sh.last_poll_utc
        : null;

  const telemetryRevision = numberOrNull(
    ws.telemetry_revision ?? sh.telemetry_revision,
  );

  const uiStale = isPullAgeStale("active", lastPullUtc ?? null, pullHz, nowMs);
  const uiPullFreshness: FreshnessView = {
    label: uiStale ? "UI pull: stale" : "UI pull: current",
    tone: uiStale ? "warn" : "ok",
    detail: lastPullUtc
      ? `Last browser pull ${formatLastPullAge(lastPullUtc, nowMs)}.`
      : "No successful telemetry pull yet.",
  };

  let telemetryFreshness = healthFreshness(telemetryHealth, "telemetry");
  if (bridgeLastPollUtc) {
    telemetryFreshness = {
      ...telemetryFreshness,
      detail: [
        telemetryFreshness.detail,
        `Adapter poll ${formatLastPullAge(bridgeLastPollUtc, nowMs)}.`,
      ]
        .filter(Boolean)
        .join(" "),
    };
  }

  const syncFreshness = healthFreshness(syncHealth, "sync");
  const launchHealth = deriveLaunchHealth(sh);
  const liveFields = pickLivePollFields(sh);
  const liveProfileActive =
    profile === "live_adapter" ||
    requestedRuntimeProfile === "live" ||
    liveFields.runtimeProfile === "live";
  const livePollFreshness = deriveLivePollFreshness(
    liveFields.lastLivePollUtc,
    liveFields.liveBackgroundPollHz,
    nowMs,
  );
  const mirrorFreshness = deriveMirrorFreshness({
    mirrorSnapshot: entityPoseMirrorSnapshot,
    worldSummaryPayload: ws,
    sessionHealthPayload: sh,
    connected,
    liveBackgroundPollHz: liveFields.liveBackgroundPollHz,
    nowMs,
  });
  const sessionState =
    typeof sh.state === "string" ? sh.state : "unknown";
  const commandHealth = deriveLiveCommandHealth({
    connected,
    requestedRuntimeProfile,
    sessionState,
    editingEnabled,
    sessionHealthPayload: sh,
    livePreflightOk,
  });

  return {
    profile,
    profileTitle: meta.title,
    profileDescription: meta.description,
    requestedProfile: requestedRuntimeProfile,
    requestedProfileLabel: requestedRuntimeProfile
      ? sessionRuntimeProfileLabel(requestedRuntimeProfile)
      : null,
    requestedProfileGovernance: requestedRuntimeProfile
      ? sessionRuntimeProfileGovernance(requestedRuntimeProfile)
      : null,
    runtimeTypeLabel: meta.runtimeTypeLabel,
    adapterMode,
    adapterAlive: adapterFields.adapterAlive,
    adapterPid: adapterFields.adapterPid,
    stubAlive: adapterFields.stubAlive,
    adapterEntityCount: adapterFields.adapterEntityCount,
    launchHealth,
    telemetryHealth,
    syncHealth,
    telemetryRevision,
    bridgeLastPollUtc,
    liveProfileActive,
    liveBackgroundPollHz: liveFields.liveBackgroundPollHz,
    lastLivePollUtc: liveFields.lastLivePollUtc,
    livePollFreshness,
    maintainerSmokeHint: liveProfileActive ? LIVE_MAINTAINER_SMOKE_HINT : null,
    mirrorFreshness,
    commandHealth,
    telemetryFreshness,
    syncFreshness,
    uiPullFreshness,
  };
}

export function profileShellClass(profile: RuntimeProfile): string {
  switch (profile) {
    case "live_adapter":
      return "border-emerald-700/70 bg-emerald-950/30";
    case "mock_adapter":
      return "border-cyan-700/70 bg-cyan-950/30";
    default:
      return "border-slate-600/80 bg-slate-900/50";
  }
}

export function profileAccentClass(profile: RuntimeProfile): string {
  switch (profile) {
    case "live_adapter":
      return "text-emerald-200";
    case "mock_adapter":
      return "text-cyan-200";
    default:
      return "text-slate-200";
  }
}
