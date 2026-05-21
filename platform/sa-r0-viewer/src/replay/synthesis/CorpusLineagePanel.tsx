import { useEffect, useState } from "react";
import { useSweepStore } from "../useSweepStore";
import { corpusEntryForSweep, loadReplayCorpusIndex } from "./loadSynthesis";
import type { ReplayCorpusIndex } from "./synthesisSchema";

function truncateRev(rev: string | undefined): string {
  if (!rev) return "—";
  return rev.length > 12 ? `${rev.slice(0, 12)}…` : rev;
}

export function CorpusLineagePanel() {
  const sweep = useSweepStore((s) => s.sweep);
  const sweepId = sweep?.sweep_id;
  const corpusRef = sweep?.corpus_ref;
  const [index, setIndex] = useState<ReplayCorpusIndex | null>(null);
  const [open, setOpen] = useState(false);

  useEffect(() => {
    loadReplayCorpusIndex()
      .then(setIndex)
      .catch(() => setIndex(null));
  }, []);

  if (!index) return null;

  const entry =
    corpusRef?.entry_id != null
      ? index.entries.find((e) => e.entry_id === corpusRef.entry_id)
      : sweepId
        ? corpusEntryForSweep(index, sweepId)
        : undefined;

  const parentIds = corpusRef?.lineage_parent_ids ?? entry?.lineage_parent_ids ?? [];
  if (!entry && parentIds.length === 0) return null;

  const indexRev = index.index_revision ?? "";
  const refStale =
    corpusRef?.index_revision != null &&
    corpusRef.index_revision !== "" &&
    corpusRef.index_revision !== indexRev;

  return (
    <section className="rounded border border-cyan-900/40 bg-cyan-950/20 p-3 text-sm">
      <button
        type="button"
        className="mb-1 flex w-full items-center justify-between font-semibold text-cyan-100"
        onClick={() => setOpen((v) => !v)}
      >
        <span>Corpus lineage</span>
        <span className="text-xs text-slate-400">{open ? "−" : "+"}</span>
      </button>
      <p className="mb-2 text-xs text-slate-400">
        {index.governance?.notice ??
          "Structural artifact derivation — not operational verification."}
      </p>
      {refStale && (
        <p className="mb-2 rounded border border-amber-900/50 bg-amber-950/30 px-2 py-1 text-[10px] text-amber-200">
          Index revision differs from artifact corpus_ref — maintainer regen may be needed.
        </p>
      )}
      {open && (
        <>
          <p className="mb-2 text-xs text-slate-300">
            Corpus: <code className="text-cyan-200">{index.corpus_id}</code>
            {" · "}
            index rev <code className="text-cyan-200">{truncateRev(indexRev)}</code>
            {entry ? (
              <>
                {" "}
                · entry <code className="text-cyan-200">{entry.entry_id}</code>
              </>
            ) : null}
          </p>
          {entry ? (
            <p className="mb-2 text-[10px] text-slate-500">
              {entry.entry_kind} · <span className="break-all">{entry.primary_artifact_path}</span>
            </p>
          ) : null}
          {parentIds.length > 0 ? (
            <ul className="list-inside list-disc space-y-1 text-xs text-slate-300">
              {parentIds.map((pid) => (
                <li key={pid}>
                  <span className="text-slate-500">parent</span> {pid}
                </li>
              ))}
            </ul>
          ) : (
            <p className="text-xs text-slate-500">No lineage parents (root artifact).</p>
          )}
          <p className="mt-2 text-[10px] text-slate-500">
            Offline reproducibility: SHA256 manifests detect fixture drift only, not tactical
            validity.
          </p>
          {(index.governance?.anti_claims ?? []).slice(0, 1).map((c) => (
            <p key={c} className="mt-1 text-[10px] text-slate-500">
              {c}
            </p>
          ))}
        </>
      )}
    </section>
  );
}
