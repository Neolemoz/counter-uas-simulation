import { useEffect, useMemo, useState } from "react";
import { loadReplayCorpusIndex } from "../synthesis/loadSynthesis";
import type { ReplayCorpusDriftReport, ReplayCorpusIndex } from "../synthesis/synthesisSchema";
import { findingsForEntry } from "./corpusNavigation";
import { CorpusEvolutionBadges } from "./CorpusEvolutionBadges";
import { loadCorpusDriftReport } from "./loadCorpusAudits";

type Props = {
  entryId: string | null;
};

const SEVERITY_CLASS: Record<string, string> = {
  info: "border-slate-700 bg-slate-900/50 text-slate-300",
  warning: "border-amber-900/50 bg-amber-950/30 text-amber-200",
  error: "border-red-900/50 bg-red-950/30 text-red-200",
};

export function CorpusProvenancePanel({ entryId }: Props) {
  const [drift, setDrift] = useState<ReplayCorpusDriftReport | null>(null);
  const [index, setIndex] = useState<ReplayCorpusIndex | null>(null);
  const [open, setOpen] = useState(false);

  useEffect(() => {
    loadCorpusDriftReport()
      .then(setDrift)
      .catch(() => setDrift(null));
    loadReplayCorpusIndex()
      .then(setIndex)
      .catch(() => setIndex(null));
  }, []);

  const entryFindings = useMemo(
    () => (drift && entryId ? findingsForEntry(drift.findings, entryId) : []),
    [drift, entryId],
  );

  const selectedEntry = useMemo(
    () => (index && entryId ? index.entries.find((e) => e.entry_id === entryId) : undefined),
    [index, entryId],
  );

  if (!drift) return null;

  const indexRev = index?.index_revision ?? "";
  const driftRev = drift.index_revision ?? "";
  const revMismatch = indexRev && driftRev && indexRev !== driftRev;

  return (
    <section className="rounded border border-amber-900/30 bg-amber-950/10 p-3 text-sm">
      <button
        type="button"
        className="mb-1 flex w-full items-center justify-between font-semibold text-amber-100"
        onClick={() => setOpen((v) => !v)}
      >
        <span>Corpus provenance / drift</span>
        <span className="text-xs text-slate-400">{open ? "−" : "+"}</span>
      </button>
      <p className="mb-2 text-[10px] text-slate-400">
        {drift.governance?.notice ??
          "Maintainer drift inventory — explanatory only, not operational monitoring."}
      </p>
      <CorpusEvolutionBadges entry={selectedEntry} />
      {revMismatch && (
        <p className="mb-2 rounded border border-amber-900/50 bg-amber-950/30 px-2 py-1 text-[10px] text-amber-200">
          Drift report index revision differs from loaded corpus index — run maintainer regen.
        </p>
      )}
      {open && (
        <>
          <p className="mb-2 text-xs text-slate-300">
            Total findings: {drift.summary?.total ?? drift.findings.length}
            {drift.summary?.by_severity && (
              <>
                {" "}
                · info {drift.summary.by_severity.info ?? 0} · warning{" "}
                {drift.summary.by_severity.warning ?? 0} · error{" "}
                {drift.summary.by_severity.error ?? 0}
              </>
            )}
          </p>
          {entryId && entryFindings.length > 0 ? (
            <ul className="space-y-1">
              {entryFindings.map((f) => (
                <li
                  key={f.finding_id}
                  className={`rounded border px-2 py-1 text-[10px] ${SEVERITY_CLASS[f.severity] ?? SEVERITY_CLASS.info}`}
                >
                  <span className="font-medium">{f.kind}</span> — {f.message}
                </li>
              ))}
            </ul>
          ) : entryId ? (
            <p className="text-xs text-slate-500">No drift findings for this entry.</p>
          ) : (
            <p className="text-xs text-slate-500">Select a corpus entry to see entry-level findings.</p>
          )}
          <p className="mt-2 text-[10px] text-slate-500">
            Reproducibility: run{" "}
            <code className="text-slate-400">verify_replay_corpus_reproducibility.py</code> in maintainer
            gate (viewer does not execute regen).
          </p>
        </>
      )}
    </section>
  );
}
