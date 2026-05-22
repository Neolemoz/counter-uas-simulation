import { useEffect, useMemo, useState } from "react";
import {
  loadAsyncIntegrityReport,
  loadOrchestrationAsyncManifest,
  loadRecoveryReport,
  loadWorkerExecutionRecord,
} from "./loadOrchestrationOps";
import type {
  AsyncIntegrityReport,
  OrchestrationAsyncManifest,
  WorkerExecutionRecord,
} from "./orchestrationOpsSchema";

const STATUS_LABEL: Record<string, string> = {
  retrying: "Retrying — CLI re-queued after bounded failure",
  failed: "Failed — terminal async execution fault",
  quarantined: "Quarantined — integrity hold; blocks forward promote",
  superseded: "Superseded — replaced by newer snapshot/claim",
};

type Props = {
  manifestId: string | null;
};

export function OrchestrationAsyncPanel({ manifestId }: Props) {
  const [asyncM, setAsyncM] = useState<OrchestrationAsyncManifest | null>(null);
  const [report, setReport] = useState<AsyncIntegrityReport | null>(null);
  const [recovery, setRecovery] = useState<{ superseded?: boolean; quarantine_hold?: boolean } | null>(
    null,
  );
  const [workers, setWorkers] = useState<WorkerExecutionRecord[]>([]);

  useEffect(() => {
    if (!manifestId) {
      setAsyncM(null);
      setWorkers([]);
      return;
    }
    loadOrchestrationAsyncManifest(manifestId)
      .then(async (data) => {
        setAsyncM(data);
        if (!data?.worker_execution_refs?.length) {
          setWorkers([]);
          return;
        }
        const loaded = await Promise.all(
          data.worker_execution_refs.map((ref) => loadWorkerExecutionRecord(ref)),
        );
        setWorkers(loaded.filter((w): w is WorkerExecutionRecord => w !== null));
      })
      .catch(() => {
        setAsyncM(null);
        setWorkers([]);
      });
  }, [manifestId]);

  useEffect(() => {
    loadAsyncIntegrityReport().then(setReport).catch(() => setReport(null));
  }, []);

  useEffect(() => {
    if (!manifestId) {
      setRecovery(null);
      return;
    }
    loadRecoveryReport(manifestId)
      .then((r) =>
        setRecovery(
          r
            ? { superseded: r.superseded, quarantine_hold: r.quarantine_hold }
            : null,
        ),
      )
      .catch(() => setRecovery(null));
  }, [manifestId]);

  const scopedIssues = useMemo(() => {
    if (!manifestId || !report?.issues) return [];
    return report.issues.filter((i) => i.manifest_id === manifestId);
  }, [manifestId, report]);

  if (!manifestId) {
    return <p className="text-xs text-slate-500">Select a pack with orchestration handoff refs.</p>;
  }

  if (!asyncM) {
    return (
      <p className="text-xs text-slate-500">
        No async mirror for <span className="font-mono">{manifestId}</span>. Optional until async
        bookkeeping is recorded via CLI.
      </p>
    );
  }

  const status = asyncM.async_execution_status;

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-violet-200/90">
        ASYNC ORCHESTRATION — CLI/workers authoritative; read-only mirror
      </p>
      {status && (
        <p
          className={
            status === "quarantined"
              ? "text-amber-300/90"
              : status === "failed"
                ? "text-red-300/90"
                : "text-slate-300"
          }
        >
          {STATUS_LABEL[status] ?? status}
        </p>
      )}
      {asyncM.execution_fingerprint && (
        <p
          className="font-mono text-[10px] text-slate-600 truncate"
          title={asyncM.execution_fingerprint}
        >
          execution fp: {asyncM.execution_fingerprint.slice(0, 28)}…
        </p>
      )}
      {asyncM.replay_fingerprint && (
        <p
          className="font-mono text-[10px] text-slate-600 truncate"
          title={asyncM.replay_fingerprint}
        >
          replay fp: {asyncM.replay_fingerprint.slice(0, 28)}…
          {recovery?.superseded ? " (superseded — historical)" : ""}
        </p>
      )}
      {recovery?.quarantine_hold && (
        <p className="text-amber-300/80 text-[10px]">
          Recovery hold active — see Recovery panel for reconciliation context
        </p>
      )}
      {workers.length > 0 && (
        <div>
          <p className="text-slate-400 mb-0.5">Worker provenance (mirror)</p>
          <ul className="space-y-0.5 font-mono text-[10px] text-slate-500">
            {workers.map((w) => (
              <li key={`${w.worker_id}-${w.execution_attempt}`}>
                {w.worker_id} attempt {w.execution_attempt}
                {w.execution_fingerprint ? ` · ${w.execution_fingerprint.slice(0, 16)}…` : ""}
              </li>
            ))}
          </ul>
        </div>
      )}
      {(asyncM.async_lineage?.length ?? 0) > 0 && (
        <div>
          <p className="text-slate-400 mb-0.5">Retry / failure lineage</p>
          <ul className="space-y-0.5 text-slate-500">
            {asyncM.async_lineage!.map((ev) => (
              <li key={ev.event_id}>
                {ev.from_status ?? "—"} → {ev.to_status}
                {ev.notes ? ` (${ev.notes})` : ""}
              </li>
            ))}
          </ul>
        </div>
      )}
      {scopedIssues.length > 0 && (
        <div>
          {scopedIssues.map((i) => (
            <p key={`${i.kind}-${i.message}`} className="text-amber-300/80">
              [{i.kind}] {i.message}
            </p>
          ))}
        </div>
      )}
      {report && (
        <p className={report.ok ? "text-emerald-400/80" : "text-amber-300/80"}>
          Async corpus audit: {report.ok ? "pass" : "issues"} (explanatory)
        </p>
      )}
    </div>
  );
}
