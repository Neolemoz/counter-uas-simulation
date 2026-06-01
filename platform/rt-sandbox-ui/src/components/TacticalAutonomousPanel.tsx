import { Crosshair } from "lucide-react";
import { PanelShell } from "@/components/GovernanceChrome";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { UiEntity } from "@/editing/localEntityMirror";

const AUTONOMOUS_BANNER =
  "AUTONOMOUS LOOP — simulate sandbox auto-assign; revert to Manual anytime";

function entityLabel(entities: UiEntity[], entityId: string | null | undefined): string {
  if (!entityId) return "—";
  const ent = entities.find((e) => e.entity_id === entityId);
  if (!ent) return entityId.slice(0, 8);
  return `${ent.entity_type} ${entityId.slice(0, 8)}`;
}

export function TacticalAutonomousPanel({
  sessionId,
  editingEnabled,
  entities,
  state,
  busy,
  error,
  onPause,
  onResume,
  onReturnToManual,
}: {
  sessionId: string | null;
  editingEnabled: boolean;
  entities: UiEntity[];
  state: TacticalStatePayload | null;
  busy: boolean;
  error: string | null;
  onPause: () => void;
  onResume: () => void;
  onReturnToManual: () => void;
}) {
  const loopStatus = state?.autonomous_loop_status ?? "paused";
  const lockActive = state?.assignment_lock_active === true;

  return (
    <PanelShell title="Tactical sandbox (autonomous)" icon={Crosshair}>
      <p className="mb-2 text-xs text-amber-200/90">{AUTONOMOUS_BANNER}</p>

      {!sessionId && (
        <p className="text-xs text-slate-500">Connect a session for autonomous loop.</p>
      )}

      {sessionId && (
        <>
          <div className="mb-2 space-y-1 text-xs text-slate-300">
            <p>
              <span className="text-slate-500">Loop:</span> {loopStatus}
            </p>
            <p>
              <span className="text-slate-500">Selected target:</span>{" "}
              {entityLabel(entities, state?.selected_target_id)}
            </p>
            <p>
              <span className="text-slate-500">Assigned:</span>{" "}
              {state?.assigned_interceptor_id && state?.assigned_target_id
                ? `${entityLabel(entities, state.assigned_interceptor_id)} → ${entityLabel(entities, state.assigned_target_id)}`
                : "—"}
            </p>
            <p>
              <span className="text-slate-500">TTI (explanatory):</span>{" "}
              {state?.tti_s != null ? `${state.tti_s.toFixed(1)} s` : "—"}
            </p>
            <p>
              <span className="text-slate-500">Assignment lock:</span>{" "}
              {lockActive ? "active" : "off"}
            </p>
            <p>
              <span className="text-slate-500">Health:</span>{" "}
              {state?.tactical_health?.summary ?? "—"}
            </p>
            {state?.authority_label && (
              <p className="text-slate-600">Authority: {state.authority_label}</p>
            )}
          </div>

          <div className="flex flex-wrap gap-2">
            <button
              type="button"
              className="rounded bg-slate-700 px-2 py-1 text-xs hover:bg-slate-600 disabled:opacity-40"
              disabled={!editingEnabled || busy || loopStatus === "paused"}
              onClick={onPause}
            >
              Pause loop
            </button>
            <button
              type="button"
              className="rounded bg-emerald-800 px-2 py-1 text-xs hover:bg-emerald-700 disabled:opacity-40"
              disabled={!editingEnabled || busy || loopStatus === "running"}
              onClick={onResume}
            >
              Resume loop
            </button>
            <button
              type="button"
              className="rounded bg-amber-900 px-2 py-1 text-xs hover:bg-amber-800 disabled:opacity-40"
              disabled={!editingEnabled || busy}
              onClick={onReturnToManual}
            >
              Return to Manual
            </button>
          </div>

          {error && <p className="mt-2 text-xs text-red-400">{error}</p>}
        </>
      )}
    </PanelShell>
  );
}
