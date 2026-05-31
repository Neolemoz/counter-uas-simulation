import { Crosshair } from "lucide-react";
import { PanelShell } from "@/components/GovernanceChrome";
import type { TacticalMode, TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { UiEntity } from "@/editing/localEntityMirror";

const TACTICAL_BANNER =
  "TACTICAL SANDBOX — simulation only; not operational coordination";

export function TacticalManualPanel({
  sessionId,
  editingEnabled,
  entities,
  selectedEntityId,
  selectedDefenderId,
  selectedTargetId,
  mode,
  state,
  busy,
  error,
  targetPickActive,
  onModeChange,
  onUseSelectedInterceptor,
  onStartTargetPick,
  onAssign,
  onClear,
  onAssignTarget,
  onCancelAssignment,
}: {
  sessionId: string | null;
  editingEnabled: boolean;
  entities: UiEntity[];
  selectedEntityId: string | null;
  selectedDefenderId: string | null;
  selectedTargetId: string | null;
  mode: TacticalMode;
  state: TacticalStatePayload | null;
  busy: boolean;
  error: string | null;
  targetPickActive: boolean;
  onModeChange: (mode: TacticalMode) => void;
  onUseSelectedInterceptor: () => void;
  onStartTargetPick: () => void;
  onAssign: () => void;
  onClear: () => void;
  onAssignTarget: () => void;
  onCancelAssignment: () => void;
}) {
  const selected = entities.find((e) => e.entity_id === selectedEntityId);
  const canUseInterceptor = selected?.entity_type === "interceptor";
  const assignDisabled = mode !== "manual";

  return (
    <PanelShell title="Tactical sandbox (manual)" icon={Crosshair} className="max-h-[520px] overflow-y-auto">
      <p className="mb-3 text-[10px] uppercase text-amber-200/80">{TACTICAL_BANNER}</p>
      <div className="mb-3 flex flex-wrap gap-2 text-xs">
        <label className="flex items-center gap-1 text-slate-300">
          <input
            type="radio"
            checked={mode === "manual"}
            disabled={!editingEnabled || busy}
            onChange={() => onModeChange("manual")}
          />{" "}
          Manual
        </label>
        <label className="flex items-center gap-1 text-slate-300">
          <input
            type="radio"
            checked={mode === "assisted"}
            disabled={!editingEnabled || busy}
            onChange={() => onModeChange("assisted")}
          />{" "}
          Assisted
        </label>
        <label className="flex items-center gap-1 text-slate-300">
          <input
            type="radio"
            checked={mode === "autonomous"}
            disabled={!editingEnabled || busy}
            onChange={() => onModeChange("autonomous")}
          />{" "}
          Autonomous
        </label>
      </div>

      {!sessionId && (
        <p className="text-xs text-slate-500">Connect a session to use tactical controls.</p>
      )}

      {sessionId && (
        <>
          <div className="mb-3 grid grid-cols-2 gap-2 text-xs">
            <div className="rounded border border-slate-800 bg-slate-950/50 px-2.5 py-2">
              <div className="text-[10px] uppercase text-slate-500">Mode</div>
              <div className="mt-1 text-cyan-100">{mode}</div>
            </div>
            <div className="rounded border border-slate-800 bg-slate-950/50 px-2.5 py-2">
              <div className="text-[10px] uppercase text-slate-500">Health</div>
              <div className="mt-1 truncate text-emerald-200">
                {state?.tactical_health?.summary ?? "—"}
              </div>
            </div>
          </div>

          <details className="mb-3 rounded border border-slate-800/80 bg-slate-950/30 text-xs">
            <summary className="cursor-pointer px-3 py-2 font-semibold text-slate-300 hover:bg-slate-900/60">
              Assignment details
            </summary>
            <div className="space-y-1 border-t border-slate-800 px-3 py-2 text-slate-300">
              <p>
                <span className="text-slate-500">Interceptor:</span>{" "}
                {state?.selected_interceptor_id ?? "—"}
              </p>
              <p>
                <span className="text-slate-500">Target:</span>{" "}
                {state?.selected_target_id ?? "—"}
              </p>
              <p>
                <span className="text-slate-500">Assigned:</span>{" "}
                {state?.assigned_interceptor_id && state?.assigned_target_id
                  ? `${state.assigned_interceptor_id} → ${state.assigned_target_id}`
                  : "—"}
              </p>
              <p>
                <span className="text-slate-500">TTI:</span>{" "}
                {state?.tti_s != null ? `${state.tti_s.toFixed(2)} s` : "—"}
              </p>
            </div>
          </details>

          <div className="flex flex-wrap gap-1.5">
            <button
              type="button"
              className="rounded border border-slate-700 bg-slate-900 px-2 py-1 text-xs text-slate-200 hover:bg-slate-800 disabled:opacity-40"
              disabled={!editingEnabled || busy || !canUseInterceptor}
              onClick={onUseSelectedInterceptor}
            >
              Use selected interceptor
            </button>
            <button
              type="button"
              className={`rounded px-2 py-1 text-xs disabled:opacity-40 ${
                targetPickActive
                  ? "bg-amber-800 hover:bg-amber-700"
                  : "bg-slate-700 hover:bg-slate-600"
              }`}
              disabled={!editingEnabled || busy}
              onClick={onStartTargetPick}
            >
              {targetPickActive ? "Click target on map…" : "Select target on map"}
            </button>
            <button
              type="button"
              className="rounded border border-emerald-700/60 bg-emerald-950/50 px-2 py-1 text-xs text-emerald-100 hover:bg-emerald-900/50 disabled:opacity-40"
              disabled={
                !editingEnabled ||
                busy ||
                assignDisabled ||
                !state?.selected_interceptor_id ||
                !state?.selected_target_id
              }
              title={
                mode === "assisted"
                  ? "Use Approve in the assisted panel to commit assignments"
                  : mode === "autonomous"
                    ? "Use autonomous loop or Return to Manual"
                    : undefined
              }
              onClick={onAssign}
            >
              Assign candidate
            </button>
            <button
              type="button"
              className="rounded border border-slate-800 bg-slate-950 px-2 py-1 text-xs text-slate-300 hover:bg-slate-900 disabled:opacity-40"
              disabled={!editingEnabled || busy}
              onClick={onClear}
            >
              Clear assignment
            </button>
          </div>

          <div className="mt-3 rounded border border-cyan-800/50 bg-cyan-950/20 p-2">
            <p className="mb-2 text-[10px] font-semibold uppercase tracking-wide text-cyan-200/80">
              Human override
            </p>
            <div className="mb-2 space-y-1 text-xs text-slate-300">
              <p>
                <span className="text-slate-500">Defender:</span>{" "}
                {selectedDefenderId ?? "—"}
              </p>
              <p>
                <span className="text-slate-500">Target:</span>{" "}
                {selectedTargetId ?? "—"}
              </p>
            </div>
            <div className="flex flex-wrap gap-1.5">
              <button
                type="button"
                className="rounded border border-cyan-700/60 bg-cyan-950/50 px-2 py-1 text-xs text-cyan-100 hover:bg-cyan-900/50 disabled:opacity-40"
                disabled={
                  !editingEnabled ||
                  busy ||
                  !selectedDefenderId ||
                  !selectedTargetId
                }
                onClick={onAssignTarget}
              >
                Assign target
              </button>
              <button
                type="button"
                className="rounded border border-slate-700 bg-slate-900 px-2 py-1 text-xs text-slate-200 hover:bg-slate-800 disabled:opacity-40"
                disabled={!editingEnabled || busy || !selectedDefenderId}
                onClick={onCancelAssignment}
              >
                Cancel assignment
              </button>
            </div>
          </div>

          {error && <p className="mt-2 text-xs text-red-400">{error}</p>}
        </>
      )}
    </PanelShell>
  );
}
