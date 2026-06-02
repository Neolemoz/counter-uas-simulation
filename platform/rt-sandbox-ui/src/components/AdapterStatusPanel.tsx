import { Cpu } from "lucide-react";
import {
  ADAPTER_VISIBILITY_GOVERNANCE,
  deriveAdapterStatus,
  profileAccentClass,
  profileShellClass,
} from "@/adapter/adapterStatus";
import type { SessionRuntimeProfile } from "@/runtime/sessionRuntimeProfile";
import { PanelShell } from "./GovernanceChrome";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { StatusBadge } from "@/workstation/StatusBadge";

function Field({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex justify-between gap-3 text-xs">
      <span className="text-slate-400">{label}</span>
      <span className="font-mono text-slate-100">{value}</span>
    </div>
  );
}

function formatBool(value: boolean | null): string {
  if (value === null) return "—";
  return value ? "yes" : "no";
}

export function AdapterStatusPanel({
  sessionHealth,
  worldSummary,
  lastPullUtc,
  pullHz = 1,
  requestedRuntimeProfile = null,
  pendingRuntimeProfile = null,
}: {
  sessionHealth: ChannelSnapshot | undefined;
  worldSummary: ChannelSnapshot | undefined;
  lastPullUtc?: string | null;
  pullHz?: number;
  requestedRuntimeProfile?: SessionRuntimeProfile | null;
  pendingRuntimeProfile?: SessionRuntimeProfile | null;
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
    requestedRuntimeProfile: requestedRuntimeProfile ?? pendingRuntimeProfile,
  });

  return (
    <PanelShell title="Runtime adapter" icon={Cpu} variant="tertiary">
      <div
        className={`mb-3 rounded-lg border px-3 py-2 ${profileShellClass(status.profile)}`}
        data-testid="adapter-status-profile"
        data-runtime-profile={status.profile}
      >
        <div className="flex flex-wrap items-center gap-2">
          <span
            className={`text-sm font-semibold uppercase tracking-wide ${profileAccentClass(status.profile)}`}
          >
            {status.profileTitle}
          </span>
          <StatusBadge label={status.runtimeTypeLabel} tone="neutral" />
        </div>
        <p className="mt-1 text-[11px] leading-snug text-slate-400">
          {status.profileDescription}
        </p>
      </div>

      {(status.requestedProfileLabel || pendingRuntimeProfile) && (
        <div className="mb-3 space-y-1 rounded border border-slate-800 bg-slate-950/40 px-2 py-2 text-[11px] text-slate-500">
          <p data-testid="adapter-status-requested-profile">
            <span className="text-slate-400">Start selection: </span>
            <span className="font-medium text-slate-300">
              {status.requestedProfileLabel ??
                (pendingRuntimeProfile === "mock_adapter"
                  ? "Mock adapter"
                  : "Stub runtime")}
            </span>
          </p>
          {status.requestedProfileGovernance && (
            <p className="leading-snug">{status.requestedProfileGovernance}</p>
          )}
        </div>
      )}

      <div className="space-y-2">
        <Field label="runtime type" value={status.runtimeTypeLabel} />
        <Field label="adapter mode" value={status.adapterMode} />
        <Field label="adapter alive" value={formatBool(status.adapterAlive)} />
        <Field label="stub alive" value={formatBool(status.stubAlive)} />
        <Field label="adapter pid" value={status.adapterPid ?? "—"} />
        {status.adapterEntityCount != null && (
          <Field
            label="adapter entities"
            value={String(status.adapterEntityCount)}
          />
        )}
        {status.telemetryRevision != null && (
          <Field
            label="telemetry revision"
            value={String(status.telemetryRevision)}
          />
        )}
      </div>

      <div className="mt-3 space-y-2 border-t border-slate-800 pt-3">
        <p className="text-[10px] font-semibold uppercase tracking-wide text-slate-500">
          Freshness (read-only)
        </p>
        <div className="flex flex-wrap gap-2" data-testid="adapter-status-freshness">
          <StatusBadge
            label={status.telemetryFreshness.label}
            tone={status.telemetryFreshness.tone}
            title={status.telemetryFreshness.detail}
          />
          <StatusBadge
            label={status.syncFreshness.label}
            tone={status.syncFreshness.tone}
            title={status.syncFreshness.detail}
          />
          <StatusBadge
            label={status.uiPullFreshness.label}
            tone={status.uiPullFreshness.tone}
            title={status.uiPullFreshness.detail}
          />
        </div>
        {status.telemetryHealth && (
          <Field label="telemetry_health" value={status.telemetryHealth} />
        )}
        {status.syncHealth && (
          <Field label="sync_health" value={status.syncHealth} />
        )}
      </div>

      <p className="mt-3 text-[11px] leading-snug text-slate-500">
        {ADAPTER_VISIBILITY_GOVERNANCE}
      </p>
    </PanelShell>
  );
}
