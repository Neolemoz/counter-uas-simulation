import { useEffect, useState } from "react";
import { loadOrchestrationOpsManifest } from "./loadOrchestrationOps";
import { OPS_LADDER_STATUSES, type OrchestrationOpsManifest } from "./orchestrationOpsSchema";

const STATUS_LABEL: Record<string, string> = {
  pending: "Pending — run CLI lint / record-validation",
  validated: "Validated — manifest + mirrors current",
  queued: "Queued — frozen queue snapshot recorded",
  executed: "Executed — run bookkeeping complete",
  replay_generated: "Replay generated — bundle outputs verified",
  archived: "Archived — terminal retired state",
};

const LADDER_RANK: Record<string, number> = Object.fromEntries(
  OPS_LADDER_STATUSES.map((s, i) => [s, i]),
);

type Props = {
  manifestId: string | null;
};

export function OrchestrationLifecyclePanel({ manifestId }: Props) {
  const [ops, setOps] = useState<OrchestrationOpsManifest | null>(null);

  useEffect(() => {
    if (!manifestId) {
      setOps(null);
      return;
    }
    loadOrchestrationOpsManifest(manifestId)
      .then(setOps)
      .catch(() => setOps(null));
  }, [manifestId]);

  if (!manifestId) {
    return <p className="text-xs text-slate-500">Select a pack with orchestration handoff refs.</p>;
  }

  if (!ops) {
    return (
      <p className="text-xs text-slate-500">
        No ops mirror for <span className="font-mono">{manifestId}</span>. Run{" "}
        <span className="font-mono">promote_experiment_manifest.py --init</span> (CLI).
      </p>
    );
  }

  const status = ops.operations_status;
  const currentRank = LADDER_RANK[status] ?? -1;

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-violet-200/90">
        ORCHESTRATION OPS — CLI promotes; not live execution
      </p>
      <p className="text-slate-300">{STATUS_LABEL[status] ?? status}</p>
      <ul className="space-y-0.5">
        {OPS_LADDER_STATUSES.map((s) => {
          const rank = LADDER_RANK[s] ?? -1;
          const done = rank <= currentRank;
          return (
            <li
              key={s}
              className={done ? "text-emerald-400/80" : "text-slate-600"}
            >
              {done ? "✓" : "○"} {s}
            </li>
          );
        })}
      </ul>
      {ops.manifest_fingerprint && (
        <p className="font-mono text-[10px] text-slate-600 truncate" title={ops.manifest_fingerprint}>
          fp: {ops.manifest_fingerprint.slice(0, 24)}…
        </p>
      )}
      {ops.queue_snapshot_ref && (
        <p className="text-[10px] text-slate-500">Queue: {ops.queue_snapshot_ref}</p>
      )}
    </div>
  );
}
