import {
  sessionRuntimeProfileLabel,
  type SessionRuntimeProfile,
} from "@/runtime/sessionRuntimeProfile";
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
}: {
  connected: boolean;
  sessionState: string;
  simPaused: boolean;
  editingAllowed: boolean;
  lastError: string | null;
  connectedCount?: number;
  editingSessionId?: string | null;
  requestedRuntimeProfile?: SessionRuntimeProfile | null;
}) {
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
