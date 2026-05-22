import { useEffect, useState } from "react";
import { useSweepStore } from "../useSweepStore";
import { corpusEntryForSweep, loadReplayCorpusIndex } from "../synthesis/loadSynthesis";
import type { ReplayCorpusIndex } from "../synthesis/synthesisSchema";
import { childrenOf } from "./corpusNavigation";
import { navigateToCorpusEntryById, type NavigateHooks } from "./navigateToCorpusEntry";
import { useCorpusStore } from "./useCorpusStore";
import { CorpusBreadcrumbs } from "./CorpusBreadcrumbs";

type Props = {
  hooks: NavigateHooks;
};

function truncateRev(rev: string | undefined): string {
  if (!rev) return "—";
  return rev.length > 12 ? `${rev.slice(0, 12)}…` : rev;
}

export function CorpusLineageNavPanel({ hooks }: Props) {
  const sweep = useSweepStore((s) => s.sweep);
  const sweepId = sweep?.sweep_id;
  const corpusRef = sweep?.corpus_ref;
  const selectedEntryId = useCorpusStore((s) => s.selectedEntryId);
  const [index, setIndex] = useState<ReplayCorpusIndex | null>(null);
  const [open, setOpen] = useState(false);

  useEffect(() => {
    loadReplayCorpusIndex()
      .then(setIndex)
      .catch(() => setIndex(null));
  }, []);

  if (!index) return null;

  const entry =
    (selectedEntryId ? index.entries.find((e) => e.entry_id === selectedEntryId) : undefined) ??
    (corpusRef?.entry_id != null
      ? index.entries.find((e) => e.entry_id === corpusRef.entry_id)
      : sweepId
        ? corpusEntryForSweep(index, sweepId)
        : undefined);

  const entryId = entry?.entry_id ?? selectedEntryId;
  const parentIds = entry
    ? entry.lineage_parent_ids
    : (corpusRef?.lineage_parent_ids ?? []);
  const childIds = entryId ? childrenOf(entryId, index) : [];

  if (!entry && parentIds.length === 0 && !entryId) return null;

  const indexRev = index.index_revision ?? "";
  const refStale =
    corpusRef?.index_revision != null &&
    corpusRef.index_revision !== "" &&
    corpusRef.index_revision !== indexRev;

  const derived = (entry?.derived_from ?? []) as { ref_kind?: string; ref_id?: string }[];

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
      {corpusRef?.entry_id && entryId !== corpusRef.entry_id && (
        <button
          type="button"
          className="mb-2 text-xs text-cyan-300 underline"
          onClick={() => void navigateToCorpusEntryById(corpusRef.entry_id, hooks)}
        >
          Open corpus_ref entry
        </button>
      )}
      {open && (
        <>
          <CorpusBreadcrumbs index={index} entryId={entryId ?? null} hooks={hooks} />
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
            <div className="mb-2">
              <p className="text-[10px] uppercase text-slate-500">Parents</p>
              <ul className="list-inside list-disc space-y-1 text-xs text-slate-300">
                {parentIds.map((pid) => (
                  <li key={pid}>
                    <button
                      type="button"
                      className="text-cyan-200 hover:underline"
                      onClick={() => void navigateToCorpusEntryById(pid, hooks)}
                    >
                      {pid}
                    </button>
                  </li>
                ))}
              </ul>
            </div>
          ) : (
            <p className="mb-2 text-xs text-slate-500">No lineage parents (root artifact).</p>
          )}
          {childIds.length > 0 && (
            <div className="mb-2">
              <p className="text-[10px] uppercase text-slate-500">Children</p>
              <ul className="list-inside list-disc space-y-1 text-xs text-slate-300">
                {childIds.map((cid) => (
                  <li key={cid}>
                    <button
                      type="button"
                      className="text-cyan-200 hover:underline"
                      onClick={() => void navigateToCorpusEntryById(cid, hooks)}
                    >
                      {cid}
                    </button>
                  </li>
                ))}
              </ul>
            </div>
          )}
          {derived.length > 0 && (
            <div className="mb-2">
              <p className="text-[10px] uppercase text-slate-500">Derived from</p>
              <ul className="space-y-1 text-[10px] text-slate-400">
                {derived.map((d, i) => (
                  <li key={`${d.ref_kind}-${i}`}>
                    {d.ref_kind}: {d.ref_id}
                  </li>
                ))}
              </ul>
            </div>
          )}
          <p className="mt-2 text-[10px] text-slate-500">
            Offline reproducibility: SHA256 manifests detect fixture drift only, not tactical
            validity.
          </p>
        </>
      )}
    </section>
  );
}
