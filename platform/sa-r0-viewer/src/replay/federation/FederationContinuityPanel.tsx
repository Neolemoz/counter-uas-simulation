import { useEffect, useState } from "react";
import {
  loadFederationContinuityIndex,
  loadFederationRecoveryContinuity,
} from "./loadFederationArtifacts";
import type {
  OrchestrationFederationRecoveryContinuity,
  ReplayFederationContinuityIndex,
} from "./federationSchema";

export function FederationContinuityPanel() {
  const [continuity, setContinuity] = useState<ReplayFederationContinuityIndex | null>(null);
  const [recovery, setRecovery] = useState<OrchestrationFederationRecoveryContinuity | null>(
    null,
  );

  useEffect(() => {
    loadFederationContinuityIndex().then(setContinuity).catch(() => setContinuity(null));
    loadFederationRecoveryContinuity().then(setRecovery).catch(() => setRecovery(null));
  }, []);

  const mismatches =
    continuity?.canonical_release_pairs?.filter((p) => p.bytes_match === false) ?? [];

  return (
    <div className="space-y-3 text-xs">
      {continuity && (
        <div>
          <p className="text-slate-400">Canonical ↔ release continuity</p>
          <p className="text-slate-500">
            {continuity.shared_entry_count ?? 0} shared entries
            {mismatches.length > 0 && (
              <span className="text-amber-300/80"> · {mismatches.length} byte mismatches</span>
            )}
          </p>
          {continuity.publication_chain_head && (
            <p className="font-mono text-[10px] text-slate-600">
              Publication head: {continuity.publication_chain_head.artifact_path}
            </p>
          )}
        </div>
      )}
      {recovery && (
        <div>
          <p className="text-slate-400">Async recovery federation scope</p>
          <p className={recovery.continuity_ok ? "text-emerald-400/80" : "text-amber-300/80"}>
            Recovery continuity: {recovery.continuity_ok ? "ok" : "issues"}
          </p>
          {(recovery.recovery_issues?.length ?? 0) > 0 && (
            <ul className="mt-1 space-y-0.5 text-slate-500">
              {recovery.recovery_issues!.slice(0, 4).map((issue, i) => (
                <li key={i}>
                  [{issue.corpus_group_id ?? "?"}] {issue.kind}: {issue.message}
                </li>
              ))}
            </ul>
          )}
        </div>
      )}
    </div>
  );
}
