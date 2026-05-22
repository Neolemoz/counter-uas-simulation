import { useEffect, useState } from "react";
import {
  loadAsyncBatchAudit,
  loadReconciliationLineageIndex,
} from "./loadOrchestrationOps";
import { loadFederationRecoveryContinuity } from "@/replay/federation/loadFederationArtifacts";
import type { OrchestrationFederationRecoveryContinuity } from "@/replay/federation/federationSchema";
import type { AsyncBatchAudit, ReconciliationLineageIndex } from "./orchestrationOpsSchema";

export function OrchestrationBatchReviewPanel() {
  const [batch, setBatch] = useState<AsyncBatchAudit | null>(null);
  const [index, setIndex] = useState<ReconciliationLineageIndex | null>(null);
  const [fedRecovery, setFedRecovery] =
    useState<OrchestrationFederationRecoveryContinuity | null>(null);

  useEffect(() => {
    loadAsyncBatchAudit().then(setBatch).catch(() => setBatch(null));
    loadReconciliationLineageIndex().then(setIndex).catch(() => setIndex(null));
    loadFederationRecoveryContinuity().then(setFedRecovery).catch(() => setFedRecovery(null));
  }, []);

  if (!batch && !index) {
    return (
      <p className="text-xs text-slate-500">
        No batch recovery mirrors. Run{" "}
        <span className="font-mono">sync_orchestration_mirrors.py</span> after recovery audit.
      </p>
    );
  }

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-violet-200/90">
        ASYNC BATCH REVIEW — corpus summary; no orchestration controls
      </p>
      {batch && (
        <>
          <p className={batch.ok ? "text-emerald-400/80" : "text-amber-300/80"}>
            Batch audit: {batch.ok ? "pass" : "issues"} · {batch.async_manifest_count ?? 0} async
            sidecars
          </p>
          {batch.status_counts && Object.keys(batch.status_counts).length > 0 && (
            <div>
              <p className="text-slate-400 mb-0.5">Status counts</p>
              <ul className="space-y-0.5 font-mono text-[10px] text-slate-500">
                {Object.entries(batch.status_counts).map(([st, n]) => (
                  <li key={st}>
                    {st}: {n}
                  </li>
                ))}
              </ul>
            </div>
          )}
          {(batch.failed_manifest_ids?.length ?? 0) > 0 && (
            <p className="text-red-300/80">
              Failed/retrying: {batch.failed_manifest_ids!.join(", ")}
            </p>
          )}
          {(batch.quarantined_manifest_ids?.length ?? 0) > 0 && (
            <p className="text-amber-300/80">
              Quarantined: {batch.quarantined_manifest_ids!.join(", ")}
            </p>
          )}
        </>
      )}
      {index && (
        <div>
          <p className="text-slate-400 mb-0.5">Reconciliation lineage index</p>
          {(index.retry_groups?.length ?? 0) > 0 && (
            <p className="text-slate-500">
              Retry groups: {index.retry_groups!.map((g) => g.manifest_id).join(", ")}
            </p>
          )}
          {(index.supersede_edges?.length ?? 0) > 0 && (
            <ul className="space-y-0.5 text-slate-500">
              {index.supersede_edges!.map((e, i) => (
                <li key={`sup-${i}`}>
                  superseded: {e.from_manifest_id}
                  {e.notes ? ` (${e.notes})` : ""}
                </li>
              ))}
            </ul>
          )}
          {(index.quarantine_holds?.length ?? 0) > 0 && (
            <p className="text-amber-300/80">
              Holds:{" "}
              {index.quarantine_holds!.map((h) => {
                const scope = (h as { corpus_group_id?: string }).corpus_group_id;
                return scope ? `${h.manifest_id}@${scope}` : h.manifest_id;
              }).join(", ")}
            </p>
          )}
        </div>
      )}
      {fedRecovery && (
        <div className="rounded border border-slate-800 bg-slate-900/50 p-2">
          <p className="text-slate-400 mb-0.5">Federation recovery continuity (F2A)</p>
          <p className={fedRecovery.continuity_ok ? "text-emerald-400/80" : "text-amber-300/80"}>
            {fedRecovery.continuity_ok ? "continuity ok" : "continuity issues"} ·{" "}
            {fedRecovery.federation_id}
          </p>
          {(fedRecovery.corpus_group_scopes?.length ?? 0) > 0 && (
            <p className="text-[10px] text-slate-600">
              Scopes:{" "}
              {fedRecovery.corpus_group_scopes!.map((s) => s.corpus_group_id).join(", ")}
            </p>
          )}
        </div>
      )}
    </div>
  );
}
