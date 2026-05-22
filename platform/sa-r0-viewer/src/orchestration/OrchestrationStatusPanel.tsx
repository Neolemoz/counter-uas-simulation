import { useEffect, useState } from "react";
import {
  loadOrchestrationIndex,
  loadOrchestrationQueue,
  readOrchestrationQueueFromUrl,
} from "./loadOrchestration";
import type { ExperimentRunQueue } from "./orchestrationSchema";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import {
  navigateFromQueueJob,
  navigateToScenarioPack,
  syncOrchestrationQueueUrl,
} from "@/navigation/experimentNavigation";

const STATUS_CLASS: Record<string, string> = {
  completed: "text-emerald-400",
  failed: "text-red-400",
  running: "text-cyan-400",
  pending: "text-slate-500",
  skipped: "text-slate-500",
  dry_run: "text-violet-300",
};

type Props = {
  hooks?: ExperimentNavHooks;
};

export function OrchestrationStatusPanel({ hooks }: Props) {
  const [queue, setQueue] = useState<ExperimentRunQueue | null>(null);
  const [queueIds, setQueueIds] = useState<string[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(readOrchestrationQueueFromUrl());
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    loadOrchestrationIndex()
      .then((entries) => {
        const ids = entries.filter((e) => e.kind === "queues").map((e) => e.id);
        setQueueIds(ids);
        setSelectedId((prev) => prev ?? readOrchestrationQueueFromUrl() ?? ids[0] ?? null);
      })
      .finally(() => setLoading(false));
  }, []);

  useEffect(() => {
    if (!selectedId) {
      setQueue(null);
      return;
    }
    syncOrchestrationQueueUrl(selectedId);
    loadOrchestrationQueue(selectedId)
      .then(setQueue)
      .catch(() => setQueue(null));
  }, [selectedId]);

  const onJobNavigate = (job: ExperimentRunQueue["jobs"][number]) => {
    if (!hooks) return;
    void navigateFromQueueJob(job, hooks);
  };

  const canNavigate = (job: ExperimentRunQueue["jobs"][number]) =>
    Boolean(
      hooks &&
        (job.scenario_pack_id ||
          job.provenance?.corpus_ref ||
          job.provenance?.bundle_path),
    );

  if (loading) {
    return <p className="text-xs text-slate-500">Loading orchestration mirror…</p>;
  }

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-amber-200/90">
        {queue?.governance_banner ?? "ORCHESTRATION MIRROR — not live execution state"}
      </p>
      {queueIds.length > 1 && (
        <label className="block text-slate-400">
          Queue snapshot
          <select
            className="mt-1 w-full rounded border border-slate-600 bg-slate-800 px-2 py-1 text-slate-200"
            value={selectedId ?? ""}
            onChange={(e) => setSelectedId(e.target.value)}
          >
            {queueIds.map((id) => (
              <option key={id} value={id}>
                {id}
              </option>
            ))}
          </select>
        </label>
      )}
      {queue ? (
        <>
          <p className="text-slate-500">
            Manifest: <span className="font-mono text-slate-400">{queue.manifest_id}</span>
          </p>
          <ul className="space-y-1">
            {queue.jobs.map((job) => (
              <li key={job.job_id} className="rounded border border-slate-700 bg-slate-950/50 px-2 py-1">
                {canNavigate(job) ? (
                  <button
                    type="button"
                    className="w-full text-left hover:bg-slate-900/50"
                    onClick={() => onJobNavigate(job)}
                  >
                    <JobRow job={job} />
                    <p className="mt-0.5 text-[10px] text-cyan-400/80">Open linked replay artifact →</p>
                  </button>
                ) : (
                  <JobRow job={job} />
                )}
                {job.scenario_pack_id && hooks ? (
                  <button
                    type="button"
                    className="mt-1 text-[10px] text-cyan-300 hover:text-cyan-200"
                    onClick={() =>
                      void navigateToScenarioPack(job.scenario_pack_id!, hooks)
                    }
                  >
                    Load pack {job.scenario_pack_id}
                  </button>
                ) : null}
              </li>
            ))}
          </ul>
          {queue.steps && queue.steps.length > 0 && (
            <details className="rounded border border-slate-800 bg-slate-900/50 p-2">
              <summary className="cursor-pointer text-slate-400">Step timeline</summary>
              <ol className="mt-2 max-h-40 space-y-1 overflow-y-auto pl-3">
                {queue.steps.map((step, i) => (
                  <li key={`${step.step_id}-${i}`} className="list-decimal text-[10px] text-slate-500">
                    <span className="text-slate-300">{step.step_id}</span> ({step.step_type}) —{" "}
                    <span className={STATUS_CLASS[step.status] ?? ""}>{step.status}</span>
                  </li>
                ))}
              </ol>
            </details>
          )}
        </>
      ) : (
        <p className="text-slate-500">No queue snapshot selected. Run offline: run_experiment_queue.py</p>
      )}
      <p className="text-[10px] text-slate-600">
        Read-only mirror. CLI/CI runs jobs; viewer does not launch simulation.
      </p>
    </div>
  );
}

function JobRow({ job }: { job: ExperimentRunQueue["jobs"][number] }) {
  return (
    <>
      <span className="font-mono text-slate-300">{job.job_id}</span>{" "}
      <span className={STATUS_CLASS[job.status] ?? "text-slate-400"}>{job.status}</span>
      {job.phase ? <span className="text-slate-500"> · {job.phase}</span> : null}
      {job.error_hint ? (
        <p className="mt-0.5 text-[10px] text-red-300/80">{job.error_hint}</p>
      ) : null}
    </>
  );
}
