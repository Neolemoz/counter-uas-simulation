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
}: {
  sessionId: string | null;
  editingEnabled: boolean;
  entities: UiEntity[];
  selectedEntityId: string | null;
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
}) {
  const selected = entities.find((e) => e.entity_id === selectedEntityId);
  const canUseInterceptor = selected?.entity_type === "interceptor";
  const assignDisabled = mode !== "manual";

  return (
    <PanelShell title="Tactical sandbox (manual)">
      <p className="mb-2 text-xs text-amber-200/90">{TACTICAL_BANNER}</p>
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
          <div className="mb-2 space-y-1 text-xs text-slate-300">
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
              <span className="text-slate-500">TTI (explanatory):</span>{" "}
              {state?.tti_s != null ? `${state.tti_s.toFixed(2)} s` : "—"}
            </p>
            <p>
              <span className="text-slate-500">Health:</span>{" "}
              {state?.tactical_health?.summary ?? "—"}
            </p>
          </div>

          <div className="flex flex-wrap gap-2">
            <button
              type="button"
              className="rounded bg-slate-700 px-2 py-1 text-xs hover:bg-slate-600 disabled:opacity-40"
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
              className="rounded bg-emerald-800 px-2 py-1 text-xs hover:bg-emerald-700 disabled:opacity-40"
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
              className="rounded bg-slate-800 px-2 py-1 text-xs hover:bg-slate-700 disabled:opacity-40"
              disabled={!editingEnabled || busy}
              onClick={onClear}
            >
              Clear assignment
            </button>
          </div>

          {error && <p className="mt-2 text-xs text-red-400">{error}</p>}
        </>
      )}
    </PanelShell>
  );
}
