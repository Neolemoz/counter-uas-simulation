import {
  sessionRuntimeProfileLabel,
  type SessionRuntimeProfile,
} from "@/runtime/sessionRuntimeProfile";
import {
  deriveLiveCommandHealth,
  LIVE_RUNTIME_CONNECTED_COPY,
} from "@/runtime/liveCommandHealth";
import { isLiveRuntimeProfile } from "@/runtime/liveSessionUx";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { StatusBadge, type StatusBadgeTone } from "./StatusBadge";

function lifecycleTone(state: string): StatusBadgeTone {
  switch (state) {
    case "running":
      return "ok";
    case "paused":
      return "warn";
    case "stopped":
      return "neutral";
    case "captured":
      return "ok";
    case "failed":
    case "runtime_crashed":
      return "error";
    default:
      return "neutral";
  }
}

export function SessionWorkflowStrip({
  connected,
  sessionState,
  simPaused,
  editingAllowed,
  lastError,
  connectedCount,
  editingSessionId,
  requestedRuntimeProfile,
  sessionHealth,
  livePreflightOk = null,
}: {
  connected: boolean;
  sessionState: string;
  simPaused: boolean;
  editingAllowed: boolean;
  lastError: string | null;
  connectedCount?: number;
  editingSessionId?: string | null;
  requestedRuntimeProfile?: SessionRuntimeProfile | null;
  sessionHealth?: ChannelSnapshot;
  livePreflightOk?: boolean | null;
}) {
  const commandHealth = deriveLiveCommandHealth({
    connected,
    requestedRuntimeProfile,
    sessionState,
    editingEnabled: editingAllowed,
    sessionHealthPayload: sessionHealth?.payload as
      | Record<string, unknown>
      | undefined,
    livePreflightOk,
  });
  const liveConnected =
    connected && isLiveRuntimeProfile(requestedRuntimeProfile);

  return (
    <div
      role="status"
      aria-label="Session workflow"
      className="flex flex-wrap items-center gap-2 rounded-lg border border-slate-700/80 bg-slate-900/60 px-3 py-2"
    >
      <StatusBadge
        label={connected ? "Connected" : "Disconnected"}
        tone={connected ? "ok" : "neutral"}
        title="Loopback bridge session"
      />
      {connected && connectedCount != null && (
        <StatusBadge
          label={`sessions: ${connectedCount}/3`}
          tone="neutral"
          title="Connected workspace slots (local cap 3)"
        />
      )}
      {connected && requestedRuntimeProfile && (
        <StatusBadge
          label={sessionRuntimeProfileLabel(requestedRuntimeProfile)}
          tone={requestedRuntimeProfile === "mock_adapter" ? "ok" : "neutral"}
          title="Runtime profile selected at session start (read-only)"
        />
      )}
      {liveConnected && (
        <span data-testid="workflow-live-connected">
          <StatusBadge
            label="Live runtime connected"
            tone="ok"
            title={LIVE_RUNTIME_CONNECTED_COPY}
          />
        </span>
      )}
      {commandHealth && (
        <span data-testid="workflow-command-health">
          <StatusBadge
            label={commandHealth.label}
            tone={commandHealth.tone}
            title={commandHealth.detail}
          />
        </span>
      )}
      {connected && editingSessionId && (
        <StatusBadge
          label={`edit: ${editingSessionId.slice(0, 8)}`}
          tone="neutral"
          title="Bridge editing lock holder"
        />
      )}
      {connected && (
        <>
          <StatusBadge
            label={`lifecycle: ${sessionState}`}
            tone={lifecycleTone(sessionState)}
          />
          {simPaused && (
            <StatusBadge label="sim paused" tone="warn" title="clock_mirror.paused" />
          )}
          <StatusBadge
            label={editingAllowed ? "editing enabled" : "editing blocked"}
            tone={editingAllowed ? "ok" : "neutral"}
          />
        </>
      )}
      {lastError && (
        <StatusBadge label={`pull/command fault`} tone="error" title={lastError} />
      )}
    </div>
  );
}
