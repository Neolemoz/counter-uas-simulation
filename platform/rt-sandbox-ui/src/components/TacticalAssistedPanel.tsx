import { PanelShell } from "@/components/GovernanceChrome";
import type {
  TacticalRecommendationPayload,
  TacticalStatePayload,
} from "@/bridge/tacticalCommands";
import type { UiEntity } from "@/editing/localEntityMirror";

const ASSISTED_BANNER =
  "ASSISTED SANDBOX — recommendations require your approval before assignment";

function entityLabel(entities: UiEntity[], entityId: string | null | undefined): string {
  if (!entityId) return "—";
  const ent = entities.find((e) => e.entity_id === entityId);
  if (!ent) return entityId.slice(0, 8);
  return `${ent.entity_type} ${entityId.slice(0, 8)}`;
}

export function TacticalAssistedPanel({
  sessionId,
  editingEnabled,
  entities,
  state,
  recommendation,
  busy,
  error,
  onRequestRecommendation,
  onApprove,
  onReject,
}: {
  sessionId: string | null;
  editingEnabled: boolean;
  entities: UiEntity[];
  state: TacticalStatePayload | null;
  recommendation: TacticalRecommendationPayload | null;
  busy: boolean;
  error: string | null;
  onRequestRecommendation: () => void;
  onApprove: () => void;
  onReject: () => void;
}) {
  const recId = recommendation?.recommendation_id;
  const feasible = recommendation?.feasibility?.feasible === true;
  const iid = recommendation?.recommended_interceptor_id;
  const tid = recommendation?.recommended_target_id;

  return (
    <PanelShell title="Tactical sandbox (assisted)">
      <p className="mb-2 text-xs text-amber-200/90">{ASSISTED_BANNER}</p>

      {!sessionId && (
        <p className="text-xs text-slate-500">
          Connect a session to use assisted recommendations.
        </p>
      )}

      {sessionId && (
        <>
          <div className="mb-2 rounded border border-slate-700 bg-slate-900/60 p-2 text-xs text-slate-300">
            <p className="mb-1 font-medium text-slate-200">Recommended</p>
            {feasible && iid && tid ? (
              <>
                <p>
                  {entityLabel(entities, iid)} → {entityLabel(entities, tid)}
                </p>
                <p className="mt-1 text-slate-400">
                  TTI (explanatory):{" "}
                  {recommendation?.tti_s != null
                    ? `${recommendation.tti_s.toFixed(1)} s`
                    : "—"}
                </p>
                <p className="text-slate-500">{recommendation?.explanation}</p>
              </>
            ) : (
              <p className="text-slate-500">
                {recommendation?.explanation ??
                  "Request a recommendation to review assignment options."}
              </p>
            )}
            <p className="mt-1 text-slate-600">
              Health: {recommendation?.tactical_health?.summary ?? state?.tactical_health?.summary ?? "—"}
            </p>
            {recommendation?.authority_label && (
              <p className="mt-1 text-slate-600">
                Authority: {recommendation.authority_label}
              </p>
            )}
          </div>

          <div className="flex flex-wrap gap-2">
            <button
              type="button"
              className="rounded bg-slate-700 px-2 py-1 text-xs hover:bg-slate-600 disabled:opacity-40"
              disabled={!editingEnabled || busy}
              onClick={onRequestRecommendation}
            >
              Refresh recommendation
            </button>
            <button
              type="button"
              className="rounded bg-emerald-800 px-2 py-1 text-xs hover:bg-emerald-700 disabled:opacity-40"
              disabled={!editingEnabled || busy || !recId || !feasible}
              onClick={onApprove}
            >
              Approve recommendation
            </button>
            <button
              type="button"
              className="rounded bg-slate-800 px-2 py-1 text-xs hover:bg-slate-700 disabled:opacity-40"
              disabled={!editingEnabled || busy || !recId}
              onClick={onReject}
            >
              Reject recommendation
            </button>
          </div>

          {error && <p className="mt-2 text-xs text-red-400">{error}</p>}
        </>
      )}
    </PanelShell>
  );
}
