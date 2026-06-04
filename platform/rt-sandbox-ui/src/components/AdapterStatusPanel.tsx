import { Cpu } from "lucide-react";
import {
  ADAPTER_VISIBILITY_GOVERNANCE,
  deriveAdapterStatus,
  profileAccentClass,
  profileShellClass,
} from "@/adapter/adapterStatus";
import {
  livePreflightRows,
  livePreflightSummary,
  type LivePreflightResult,
} from "@/runtime/livePreflight";
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

function pendingProfileLabel(profile: SessionRuntimeProfile): string {
  if (profile === "mock_adapter") return "Mock adapter";
  if (profile === "live") return "Live Gazebo";
  return "Stub runtime";
}

export function AdapterStatusPanel({
  sessionHealth,
  worldSummary,
  entityPoseMirror,
  connected = true,
  editingEnabled = true,
  livePreflightOk = null,
  lastPullUtc,
  pullHz = 1,
  requestedRuntimeProfile = null,
  pendingRuntimeProfile = null,
  livePreflight = null,
  preflightLoading = false,
  preflightError = null,
}: {
  sessionHealth: ChannelSnapshot | undefined;
  worldSummary: ChannelSnapshot | undefined;
  entityPoseMirror?: ChannelSnapshot | undefined;
  connected?: boolean;
  editingEnabled?: boolean;
  livePreflightOk?: boolean | null;
  lastPullUtc?: string | null;
  pullHz?: number;
  requestedRuntimeProfile?: SessionRuntimeProfile | null;
  pendingRuntimeProfile?: SessionRuntimeProfile | null;
  livePreflight?: LivePreflightResult | null;
  preflightLoading?: boolean;
  preflightError?: string | null;
}) {
  const status = deriveAdapterStatus({
    sessionHealthPayload: sessionHealth?.payload as
      | Record<string, unknown>
      | undefined,
    worldSummaryPayload: worldSummary?.payload as
      | Record<string, unknown>
      | undefined,
    entityPoseMirrorSnapshot: entityPoseMirror,
    connected,
    lastPullUtc,
    pullHz,
    requestedRuntimeProfile: requestedRuntimeProfile ?? pendingRuntimeProfile,
    editingEnabled,
    livePreflightOk,
  });

  const showPreflight =
    pendingRuntimeProfile === "live" ||
    requestedRuntimeProfile === "live" ||
    status.profile === "live_adapter";

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
                (pendingRuntimeProfile
                  ? pendingProfileLabel(pendingRuntimeProfile)
                  : "Stub runtime")}
            </span>
          </p>
          {status.requestedProfileGovernance && (
            <p className="leading-snug">{status.requestedProfileGovernance}</p>
          )}
        </div>
      )}

      {showPreflight && (
        <div
          className="mb-3 space-y-2 rounded border border-emerald-900/50 bg-emerald-950/20 px-2 py-2"
          data-testid="adapter-live-preflight"
        >
          <div className="flex flex-wrap items-center gap-2">
            <p className="text-[10px] font-semibold uppercase tracking-wide text-emerald-300/90">
              Live preflight
            </p>
            {preflightLoading ? (
              <StatusBadge label="checking…" tone="neutral" />
            ) : livePreflight?.ok ? (
              <StatusBadge label="ready" tone="ok" />
            ) : (
              <StatusBadge label="unavailable" tone="warn" />
            )}
          </div>
          {livePreflightRows(livePreflight).map((row) => (
            <Field
              key={row.id}
              label={row.label}
              value={row.ok ? "ok" : "missing"}
            />
          ))}
          <p className="text-[11px] leading-snug text-slate-500">
            {preflightError ?? livePreflightSummary(livePreflight)}
          </p>
        </div>
      )}

      {status.liveProfileActive && (
        <div
          className="mb-3 space-y-2 rounded border border-emerald-900/40 bg-slate-950/30 px-2 py-2"
          data-testid="adapter-live-status"
        >
          <div className="flex flex-wrap items-center gap-2">
            <p className="text-[10px] font-semibold uppercase tracking-wide text-emerald-300/90">
              Live session
            </p>
            <StatusBadge label="live profile active" tone="ok" />
            {status.launchHealth && (
              <StatusBadge label={status.launchHealth} tone="neutral" />
            )}
            {status.commandHealth && (
              <span data-testid="adapter-command-health">
                <StatusBadge
                  label={status.commandHealth.label}
                  tone={status.commandHealth.tone}
                  title={status.commandHealth.detail}
                />
              </span>
            )}
          </div>
          {status.liveBackgroundPollHz != null && (
            <Field
              label="live poll rate"
              value={`${status.liveBackgroundPollHz} Hz`}
            />
          )}
          <Field
            label="last live poll (bridge)"
            value={status.lastLivePollUtc ?? "—"}
          />
          <div className="flex flex-wrap gap-2" data-testid="adapter-live-poll-freshness">
            <StatusBadge
              label={status.livePollFreshness.label}
              tone={status.livePollFreshness.tone}
              title={status.livePollFreshness.detail}
            />
            <span data-testid="adapter-mirror-freshness">
              <StatusBadge
                label={status.mirrorFreshness.label}
                tone={status.mirrorFreshness.tone}
                title={status.mirrorFreshness.detail}
              />
            </span>
          </div>
          {status.maintainerSmokeHint && (
            <p
              className="text-[11px] leading-snug text-slate-500"
              data-testid="adapter-maintainer-smoke-hint"
            >
              {status.maintainerSmokeHint}
            </p>
          )}
        </div>
      )}

      <div className="space-y-2">
        <Field label="runtime type" value={status.runtimeTypeLabel} />
        <Field label="adapter mode" value={status.adapterMode} />
        <Field label="adapter alive" value={formatBool(status.adapterAlive)} />
        {status.launchHealth && (
          <Field label="launch health" value={status.launchHealth} />
        )}
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
          <span data-testid="adapter-mirror-freshness-summary">
            <StatusBadge
              label={status.mirrorFreshness.label}
              tone={status.mirrorFreshness.tone}
              title={status.mirrorFreshness.detail}
            />
          </span>
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
