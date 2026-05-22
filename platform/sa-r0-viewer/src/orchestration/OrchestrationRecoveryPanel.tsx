import { useEffect, useMemo, useState } from "react";
import {
  loadOrchestrationAsyncManifest,
  loadQueueClaimToken,
  loadRecoveryReport,
} from "./loadOrchestrationOps";
import type {
  OrchestrationAsyncManifest,
  OrchestrationRecoveryReport,
  QueueClaimToken,
} from "./orchestrationOpsSchema";

type Props = {
  manifestId: string | null;
};

export function OrchestrationRecoveryPanel({ manifestId }: Props) {
  const [asyncM, setAsyncM] = useState<OrchestrationAsyncManifest | null>(null);
  const [recovery, setRecovery] = useState<OrchestrationRecoveryReport | null>(null);
  const [claim, setClaim] = useState<QueueClaimToken | null>(null);

  useEffect(() => {
    if (!manifestId) {
      setAsyncM(null);
      setRecovery(null);
      setClaim(null);
      return;
    }
    loadOrchestrationAsyncManifest(manifestId).then(setAsyncM).catch(() => setAsyncM(null));
    loadRecoveryReport(manifestId).then(setRecovery).catch(() => setRecovery(null));
  }, [manifestId]);

  useEffect(() => {
    const cref = asyncM?.queue_claim_ref;
    if (!cref) {
      setClaim(null);
      return;
    }
    loadQueueClaimToken(cref).then(setClaim).catch(() => setClaim(null));
  }, [asyncM?.queue_claim_ref]);

  const retryChain = useMemo(
    () => recovery?.retry_chain ?? [],
    [recovery?.retry_chain],
  );

  if (!manifestId) {
    return <p className="text-xs text-slate-500">Select a pack with orchestration handoff refs.</p>;
  }

  if (!asyncM && !recovery) {
    return (
      <p className="text-xs text-slate-500">
        No recovery mirror for <span className="font-mono">{manifestId}</span>. Run{" "}
        <span className="font-mono">audit_orchestration_recovery.py --refresh-fixtures</span>.
      </p>
    );
  }

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-amber-200/90">
        RECOVERY / RECONCILIATION — explanatory only; reconcile forward via CLI
      </p>
      {recovery?.quarantine_hold && (
        <p className="text-amber-300/90">Quarantine hold — forward promote blocked until CLI review</p>
      )}
      {recovery?.superseded && (
        <p className="text-slate-400">Superseded snapshot — read-only lineage; successor owns reconcile</p>
      )}
      {claim && (
        <div>
          <p className="text-slate-400 mb-0.5">Queue claim (mirror)</p>
          <p className="font-mono text-[10px] text-slate-500">
            {claim.claim_id} · {claim.claim_status ?? "unknown"}
            {claim.snapshot_hash ? ` · ${claim.snapshot_hash.slice(0, 20)}…` : ""}
          </p>
        </div>
      )}
      {retryChain.length > 0 && (
        <div>
          <p className="text-slate-400 mb-0.5">Retry lineage chain</p>
          <ul className="space-y-0.5 text-slate-500">
            {retryChain.map((hop, i) => (
              <li key={`hop-${i}`}>
                {String(hop.hop_type ?? "hop")}
                {hop.worker_id ? `: ${String(hop.worker_id)} attempt ${String(hop.execution_attempt)}` : ""}
                {hop.claim_status ? ` (${String(hop.claim_status)})` : ""}
                {hop.from_status != null || hop.to_status
                  ? ` · ${String(hop.from_status ?? "—")} → ${String(hop.to_status)}`
                  : ""}
              </li>
            ))}
          </ul>
        </div>
      )}
      {recovery?.replay_reconciliation && (
        <div>
          <p className="text-slate-400 mb-0.5">Replay reconciliation</p>
          <p
            className={
              recovery.replay_reconciliation.ok === false
                ? "text-amber-300/80"
                : "text-emerald-400/80"
            }
          >
            {recovery.replay_reconciliation.ok === false ? "stale or missing fingerprint" : "fingerprints aligned"}
          </p>
        </div>
      )}
      {(recovery?.recovery_issues?.length ?? 0) > 0 && (
        <div>
          {recovery!.recovery_issues!.map((i) => (
            <p key={`${i.kind}-${i.message}`} className="text-amber-300/80">
              [{i.kind}] {i.message}
            </p>
          ))}
        </div>
      )}
      {recovery && (
        <p className={recovery.recovery_continuity_ok ? "text-emerald-400/80" : "text-amber-300/80"}>
          Recovery continuity: {recovery.recovery_continuity_ok ? "pass" : "issues"} (mirror)
        </p>
      )}
    </div>
  );
}
