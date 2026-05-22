import { useEffect, useState } from "react";
import { loadOrchestrationQueue } from "./loadOrchestration";
import { loadOrchestrationOpsManifest } from "./loadOrchestrationOps";
import type { ExperimentRunQueue } from "./orchestrationSchema";

type Props = {
  manifestId: string | null;
  queueId: string | null;
};

export function OrchestrationReplayContinuityPanel({ manifestId, queueId }: Props) {
  const [queue, setQueue] = useState<ExperimentRunQueue | null>(null);
  const [opsStatus, setOpsStatus] = useState<string | null>(null);

  useEffect(() => {
    if (!manifestId) {
      setOpsStatus(null);
      return;
    }
    loadOrchestrationOpsManifest(manifestId)
      .then((ops) => setOpsStatus(ops?.operations_status ?? null))
      .catch(() => setOpsStatus(null));
  }, [manifestId]);

  useEffect(() => {
    if (!queueId) {
      setQueue(null);
      return;
    }
    loadOrchestrationQueue(queueId)
      .then(setQueue)
      .catch(() => setQueue(null));
  }, [queueId]);

  if (!queueId && !manifestId) {
    return <p className="text-xs text-slate-500">No queue or manifest context.</p>;
  }

  const jobs = queue?.jobs ?? [];
  const bundlePaths = jobs
    .map((j) => j.provenance?.bundle_path)
    .filter((p): p is string => Boolean(p));

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-violet-200/90">Replay continuity (read-only)</p>
      {opsStatus && (
        <p className="text-slate-400">
          Ops status: <span className="font-mono text-slate-200">{opsStatus}</span>
        </p>
      )}
      {queue && (
        <p className="text-slate-500">
          Queue <span className="font-mono text-slate-300">{queue.queue_id}</span> · dry_run=
          {String(queue.dry_run ?? false)}
        </p>
      )}
      {bundlePaths.length > 0 ? (
        <ul className="list-inside list-disc text-slate-400">
          {bundlePaths.map((p) => (
            <li key={p} className="font-mono text-[10px] truncate" title={p}>
              {p}
            </li>
          ))}
        </ul>
      ) : (
        <p className="text-slate-600">
          Validation-only jobs — bundle linkage appears after synthetic pipeline (CLI).
        </p>
      )}
      {opsStatus === "replay_generated" && (
        <p className="text-emerald-400/80">Replay outputs verified per ops manifest (CLI).</p>
      )}
    </div>
  );
}
