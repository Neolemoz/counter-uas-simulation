import { useState } from "react";
import { DEFAULT_PULL_HZ, MAX_PULL_HZ } from "@/telemetry/constants";

export function RefreshControls({
  pullHz,
  autoRefresh,
  pulling,
  onPullHzChange,
  onAutoRefreshChange,
  onRefresh,
}: {
  pullHz: number;
  autoRefresh: boolean;
  pulling: boolean;
  onPullHzChange: (hz: number) => void;
  onAutoRefreshChange: (enabled: boolean) => void;
  onRefresh: () => void;
}) {
  return (
    <div className="flex flex-wrap items-center gap-3 rounded-lg border border-slate-700 bg-slate-900/80 p-3">
      <button
        type="button"
        onClick={onRefresh}
        disabled={pulling}
        className="rounded bg-slate-700 px-3 py-1.5 text-sm font-medium text-slate-100 hover:bg-slate-600 disabled:opacity-50"
      >
        {pulling ? "Pulling…" : "Refresh now"}
      </button>
      <label className="flex items-center gap-2 text-sm text-slate-300">
        <input
          type="checkbox"
          checked={autoRefresh}
          onChange={(e) => onAutoRefreshChange(e.target.checked)}
          className="rounded"
        />
        Auto refresh
      </label>
      <label className="flex items-center gap-2 text-sm text-slate-300">
        Rate (Hz)
        <input
          type="number"
          min={0.1}
          max={MAX_PULL_HZ}
          step={0.1}
          value={pullHz}
          onChange={(e) =>
            onPullHzChange(
              Math.min(MAX_PULL_HZ, Math.max(0.1, Number(e.target.value) || DEFAULT_PULL_HZ)),
            )
          }
          className="w-16 rounded border border-slate-600 bg-slate-950 px-2 py-1 font-mono text-sm"
        />
      </label>
      <span className="text-xs text-slate-500">Pull-only; max {MAX_PULL_HZ} Hz</span>
    </div>
  );
}

export function CollapsibleUiDiagnostics({
  defaultOpen = false,
  ...props
}: {
  sessionId: string | null;
  subscriptionId: string | null;
  lastPullUtc: string | null;
  drainedCount: number;
  lastError: string | null;
  sessionState: string;
  lastCommand?: string;
  pendingReconcile?: boolean;
  defaultOpen?: boolean;
}) {
  const [open, setOpen] = useState(defaultOpen);
  return (
    <div className="rounded-lg border border-slate-700 bg-slate-900/60">
      <button
        type="button"
        onClick={() => setOpen((v) => !v)}
        className="flex w-full items-center justify-between px-3 py-2 text-left text-xs font-semibold text-slate-300 hover:bg-slate-800/50"
      >
        UI diagnostics
        <span className="text-slate-500">{open ? "▾" : "▸"}</span>
      </button>
      {open && (
        <div className="border-t border-slate-700 px-3 pb-3">
          <UiDiagnostics {...props} bare />
        </div>
      )}
    </div>
  );
}

export function UiDiagnostics({
  sessionId,
  subscriptionId,
  lastPullUtc,
  drainedCount,
  lastError,
  sessionState,
  lastCommand,
  pendingReconcile,
  bare = false,
}: {
  sessionId: string | null;
  subscriptionId: string | null;
  lastPullUtc: string | null;
  drainedCount: number;
  lastError: string | null;
  sessionState: string;
  lastCommand?: string;
  pendingReconcile?: boolean;
  bare?: boolean;
}) {
  return (
    <div
      className={
        bare
          ? "text-xs font-mono text-slate-400"
          : "rounded-lg border border-slate-700 bg-slate-900/60 p-3 text-xs font-mono text-slate-400"
      }
    >
      {!bare && <p className="mb-1 font-semibold text-slate-300">UI diagnostics</p>}
      <p>bridge: /v1 (Vite proxy → 127.0.0.1:18765)</p>
      <p>session_id: {sessionId ?? "—"}</p>
      <p>subscription_id: {subscriptionId ?? "—"}</p>
      <p>session_state: {sessionState}</p>
      <p>last_pull_utc: {lastPullUtc ?? "—"}</p>
      <p>last_drained_count: {drainedCount}</p>
      <p>last_command: {lastCommand ?? "—"}</p>
      <p>pending_reconcile: {pendingReconcile ? "yes" : "no"}</p>
      {lastError && <p className="text-red-400">error: {lastError}</p>}
      <p className="mt-2 text-slate-500">
        Maintainer audit: scripts/rt/rt_adapter_inspect.py telemetry-status
      </p>
    </div>
  );
}

export function BridgeConnectionBar({
  connected,
  busy,
  onConnect,
  onDisconnect,
}: {
  connected: boolean;
  busy: boolean;
  onConnect: () => void;
  onDisconnect: () => void;
}) {
  return (
    <div className="flex flex-wrap items-center gap-3">
      <button
        type="button"
        onClick={onConnect}
        disabled={connected || busy}
        className="rounded bg-emerald-800 px-3 py-1.5 text-sm font-medium text-emerald-100 hover:bg-emerald-700 disabled:opacity-50"
      >
        Start session & subscribe
      </button>
      <button
        type="button"
        onClick={onDisconnect}
        disabled={!connected || busy}
        className="rounded bg-slate-700 px-3 py-1.5 text-sm font-medium text-slate-200 hover:bg-slate-600 disabled:opacity-50"
      >
        Stop & unsubscribe
      </button>
      <span
        className={`text-sm ${connected ? "text-emerald-400" : "text-slate-500"}`}
      >
        {connected ? "Connected (loopback)" : "Not connected"}
      </span>
    </div>
  );
}

export function RuntimeControlBar({
  connected,
  editingEnabled,
  busy,
  simPaused,
  onPauseSim,
  onResumeSim,
  onSpawnDefender,
}: {
  connected: boolean;
  editingEnabled: boolean;
  busy: boolean;
  simPaused: boolean;
  onPauseSim: () => void;
  onResumeSim: () => void;
  onSpawnDefender: () => void;
}) {
  const controlsDisabled = !connected || !editingEnabled || busy;

  return (
    <div className="flex flex-wrap items-center gap-2 rounded-lg border border-slate-700/80 bg-slate-900/50 px-3 py-2">
      <span className="text-xs font-semibold uppercase tracking-wide text-slate-400">
        Runtime
      </span>
      <button
        type="button"
        onClick={onPauseSim}
        disabled={controlsDisabled || simPaused}
        className="rounded border border-amber-700/60 bg-amber-950/50 px-2.5 py-1 text-xs font-medium text-amber-100 hover:bg-amber-900/50 disabled:opacity-40"
      >
        Pause
      </button>
      <button
        type="button"
        onClick={onResumeSim}
        disabled={controlsDisabled || !simPaused}
        className="rounded border border-emerald-700/60 bg-emerald-950/50 px-2.5 py-1 text-xs font-medium text-emerald-100 hover:bg-emerald-900/50 disabled:opacity-40"
      >
        Resume
      </button>
      <button
        type="button"
        onClick={onSpawnDefender}
        disabled={controlsDisabled}
        className="rounded border border-cyan-700/60 bg-cyan-950/50 px-2.5 py-1 text-xs font-medium text-cyan-100 hover:bg-cyan-900/50 disabled:opacity-40"
      >
        Spawn defender
      </button>
    </div>
  );
}
