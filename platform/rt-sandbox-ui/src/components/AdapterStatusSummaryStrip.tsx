import {
  deriveAdapterStatus,
  profileAccentClass,
  profileShellClass,
} from "@/adapter/adapterStatus";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { StatusBadge } from "@/workstation/StatusBadge";

import type { SessionRuntimeProfile } from "@/runtime/sessionRuntimeProfile";

export function AdapterStatusSummaryStrip({
  sessionHealth,
  worldSummary,
  lastPullUtc,
  pullHz = 1,
  compact = false,
  requestedRuntimeProfile = null,
}: {
  sessionHealth: ChannelSnapshot | undefined;
  worldSummary: ChannelSnapshot | undefined;
  lastPullUtc?: string | null;
  pullHz?: number;
  compact?: boolean;
  requestedRuntimeProfile?: SessionRuntimeProfile | null;
}) {
  const status = deriveAdapterStatus({
    sessionHealthPayload: sessionHealth?.payload as
      | Record<string, unknown>
      | undefined,
    worldSummaryPayload: worldSummary?.payload as
      | Record<string, unknown>
      | undefined,
    lastPullUtc,
    pullHz,
    requestedRuntimeProfile,
  });

  return (
    <div
      className={`rounded border px-2.5 py-2 ${profileShellClass(status.profile)} ${compact ? "text-[11px]" : "text-xs"}`}
      data-testid="adapter-status-summary"
      data-runtime-profile={status.profile}
    >
      <div className="flex flex-wrap items-center gap-2">
        <span className={`font-semibold ${profileAccentClass(status.profile)}`}>
          {status.profileTitle}
        </span>
        <StatusBadge label={`mode: ${status.adapterMode}`} tone="neutral" />
        <StatusBadge
          label={status.telemetryFreshness.label}
          tone={status.telemetryFreshness.tone}
        />
        <StatusBadge
          label={status.syncFreshness.label}
          tone={status.syncFreshness.tone}
        />
      </div>
      {!compact && (
        <p className="mt-1 text-slate-500">{status.profileDescription}</p>
      )}
    </div>
  );
}
