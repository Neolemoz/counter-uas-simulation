import { useState } from "react";
import { RuntimeProfileSelector } from "@/components/RuntimeProfileSelector";
import { ENTITY_CONTROL_GOVERNANCE } from "@/entity/entityControlStates";
import { lifecycleControlStates } from "@/lifecycle/lifecycleControlStates";
import {
  LIVE_STOP_GOVERNANCE,
  isLiveRuntimeProfile,
} from "@/runtime/liveSessionUx";
import {
  sessionRuntimeProfileLabel,
  type SessionRuntimeProfile,
} from "@/runtime/sessionRuntimeProfile";
import { SCENARIO_CONTROL_GOVERNANCE } from "@/scenario/scenarioControlStates";
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
  atCapacity = false,
  sessionRuntimeProfile,
  onSessionRuntimeProfileChange,
  activeRequestedRuntimeProfile,
  onConnect,
  onDisconnect,
}: {
  connected: boolean;
  busy: boolean;
  atCapacity?: boolean;
  sessionRuntimeProfile: SessionRuntimeProfile;
  onSessionRuntimeProfileChange: (profile: SessionRuntimeProfile) => void;
  activeRequestedRuntimeProfile?: SessionRuntimeProfile | null;
  onConnect: () => void;
  onDisconnect: () => void;
}) {
  return (
    <div className="flex flex-col gap-3">
      <RuntimeProfileSelector
        value={sessionRuntimeProfile}
        onChange={onSessionRuntimeProfileChange}
        disabled={busy}
      />
      {connected && activeRequestedRuntimeProfile && (
        <p
          className="text-xs text-slate-500"
          data-testid="bridge-active-runtime-profile"
        >
          Active session runtime:{" "}
          <span className="font-medium text-slate-300">
            {sessionRuntimeProfileLabel(activeRequestedRuntimeProfile)}
          </span>
        </p>
      )}
      {connected && isLiveRuntimeProfile(activeRequestedRuntimeProfile) && (
        <p
          className="text-xs text-amber-400/90"
          data-testid="bridge-live-stop-copy"
        >
          {LIVE_STOP_GOVERNANCE}
        </p>
      )}
      <div className="flex flex-wrap items-center gap-3">
      <button
        type="button"
        onClick={onConnect}
        disabled={connected || busy || atCapacity}
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
    </div>
  );
}

export function LifecycleControlBar({
  connected,
  editingEnabled,
  busy,
  sessionState,
  requestedRuntimeProfile = null,
  onPause,
  onResume,
  onReset,
  onStopSession,
}: {
  connected: boolean;
  editingEnabled: boolean;
  busy: boolean;
  sessionState: string;
  requestedRuntimeProfile?: SessionRuntimeProfile | null;
  onPause: () => void;
  onResume: () => void;
  onReset: () => void;
  onStopSession: () => void;
}) {
  const baseDisabled = !connected || !editingEnabled || busy;
  const states = lifecycleControlStates(sessionState);
  const liveActive = isLiveRuntimeProfile(requestedRuntimeProfile);

  return (
    <div className="flex flex-col gap-2 rounded-lg border border-slate-600/80 bg-slate-900/70 px-3 py-2">
      <div className="flex flex-wrap items-center gap-2">
      <span className="text-xs font-semibold uppercase tracking-wide text-slate-400">
        Lifecycle
      </span>
      <span className="text-xs text-slate-500">({sessionState})</span>
      <button
        type="button"
        onClick={onPause}
        disabled={baseDisabled || !states.pause}
        className="rounded border border-amber-700/60 bg-amber-950/50 px-2.5 py-1 text-xs font-medium text-amber-100 hover:bg-amber-900/50 disabled:opacity-40"
      >
        Pause
      </button>
      <button
        type="button"
        onClick={onResume}
        disabled={baseDisabled || !states.resume}
        className="rounded border border-emerald-700/60 bg-emerald-950/50 px-2.5 py-1 text-xs font-medium text-emerald-100 hover:bg-emerald-900/50 disabled:opacity-40"
      >
        Resume
      </button>
      <button
        type="button"
        onClick={onReset}
        disabled={baseDisabled || !states.reset}
        className="rounded border border-sky-700/60 bg-sky-950/50 px-2.5 py-1 text-xs font-medium text-sky-100 hover:bg-sky-900/50 disabled:opacity-40"
      >
        Reset
      </button>
      <button
        type="button"
        onClick={onStopSession}
        disabled={baseDisabled || !states.stopSession}
        className="rounded border border-rose-700/60 bg-rose-950/50 px-2.5 py-1 text-xs font-medium text-rose-100 hover:bg-rose-900/50 disabled:opacity-40"
      >
        Stop session
      </button>
      </div>
      {liveActive && (
        <p
          className="text-[11px] leading-snug text-amber-400/90"
          data-testid="lifecycle-live-stop-copy"
        >
          {LIVE_STOP_GOVERNANCE}
        </p>
      )}
    </div>
  );
}

export function ScenarioControlBar({
  connected,
  applyDisabled,
  entityCount,
  onApplyScenario,
}: {
  connected: boolean;
  applyDisabled: boolean;
  entityCount: number;
  onApplyScenario: () => void;
}) {
  if (!connected) return null;

  return (
    <div
      className="flex flex-col gap-2 rounded-lg border border-slate-600/80 bg-slate-900/70 px-3 py-2"
      data-testid="scenario-control-bar"
    >
      <div className="flex flex-wrap items-center gap-2">
        <span className="text-xs font-semibold uppercase tracking-wide text-slate-400">
          Scenario
        </span>
        <button
          type="button"
          onClick={onApplyScenario}
          disabled={applyDisabled}
          data-testid="scenario-apply"
          className="rounded border border-emerald-700/60 bg-emerald-950/50 px-2.5 py-1 text-xs font-medium text-emerald-100 hover:bg-emerald-900/50 disabled:opacity-40"
        >
          Apply scenario
        </button>
        {entityCount === 0 && connected && (
          <span className="text-xs text-slate-500">Add entities in the world editor first</span>
        )}
      </div>
      <p className="text-[11px] leading-snug text-slate-500">{SCENARIO_CONTROL_GOVERNANCE}</p>
    </div>
  );
}

export function EntityControlBar({
  connected,
  controlsDisabled,
  deleteDisabled,
  selectedEntityId,
  onSpawnAttacker,
  onSpawnDefender,
  onDeleteSelected,
}: {
  connected: boolean;
  controlsDisabled: boolean;
  deleteDisabled: boolean;
  selectedEntityId: string | null;
  onSpawnAttacker: () => void;
  onSpawnDefender: () => void;
  onDeleteSelected: () => void;
}) {
  if (!connected) return null;

  return (
    <div
      className="flex flex-col gap-2 rounded-lg border border-slate-600/80 bg-slate-900/70 px-3 py-2"
      data-testid="entity-control-bar"
    >
      <div className="flex flex-wrap items-center gap-2">
        <span className="text-xs font-semibold uppercase tracking-wide text-slate-400">
          Entities
        </span>
        <button
          type="button"
          onClick={onSpawnAttacker}
          disabled={controlsDisabled}
          data-testid="entity-spawn-attacker"
          className="rounded border border-orange-700/60 bg-orange-950/50 px-2.5 py-1 text-xs font-medium text-orange-100 hover:bg-orange-900/50 disabled:opacity-40"
        >
          Spawn attacker
        </button>
        <button
          type="button"
          onClick={onSpawnDefender}
          disabled={controlsDisabled}
          data-testid="entity-spawn-defender"
          className="rounded border border-cyan-700/60 bg-cyan-950/50 px-2.5 py-1 text-xs font-medium text-cyan-100 hover:bg-cyan-900/50 disabled:opacity-40"
        >
          Spawn defender
        </button>
        <button
          type="button"
          onClick={onDeleteSelected}
          disabled={deleteDisabled}
          data-testid="entity-delete-selected"
          className="rounded border border-rose-700/60 bg-rose-950/50 px-2.5 py-1 text-xs font-medium text-rose-100 hover:bg-rose-900/50 disabled:opacity-40"
        >
          Delete selected
        </button>
        {selectedEntityId && (
          <span className="font-mono text-xs text-slate-500">{selectedEntityId}</span>
        )}
      </div>
      <p className="text-[11px] leading-snug text-slate-500">{ENTITY_CONTROL_GOVERNANCE}</p>
    </div>
  );
}

export function RuntimeControlBar({
  connected,
  editingEnabled,
  busy,
  onSpawnDefender,
}: {
  connected: boolean;
  editingEnabled: boolean;
  busy: boolean;
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
        onClick={onSpawnDefender}
        disabled={controlsDisabled}
        className="rounded border border-cyan-700/60 bg-cyan-950/50 px-2.5 py-1 text-xs font-medium text-cyan-100 hover:bg-cyan-900/50 disabled:opacity-40"
      >
        Spawn defender
      </button>
    </div>
  );
}

export function CaptureControlBar({
  connected,
  editingEnabled,
  busy,
  captureActive,
  onStartCapture,
  onStopCapture,
}: {
  connected: boolean;
  editingEnabled: boolean;
  busy: boolean;
  captureActive: boolean;
  onStartCapture: () => void;
  onStopCapture: () => void;
}) {
  const controlsDisabled = !connected || !editingEnabled || busy;

  return (
    <div className="flex flex-wrap items-center gap-2 rounded-lg border border-slate-700/80 bg-slate-900/50 px-3 py-2">
      <span className="text-xs font-semibold uppercase tracking-wide text-slate-400">
        Capture
      </span>
      <button
        type="button"
        onClick={onStartCapture}
        disabled={controlsDisabled || captureActive}
        className="rounded border border-violet-700/60 bg-violet-950/50 px-2.5 py-1 text-xs font-medium text-violet-100 hover:bg-violet-900/50 disabled:opacity-40"
      >
        Start capture
      </button>
      <button
        type="button"
        onClick={onStopCapture}
        disabled={controlsDisabled || !captureActive}
        className="rounded border border-rose-700/60 bg-rose-950/50 px-2.5 py-1 text-xs font-medium text-rose-100 hover:bg-rose-900/50 disabled:opacity-40"
      >
        Stop capture
      </button>
    </div>
  );
}
