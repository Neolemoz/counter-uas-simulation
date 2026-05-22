import type { ReplayCorpusIndex } from "../synthesis/synthesisSchema";
import { REVIEWER_CATEGORY_LABELS, ancestorChain } from "./corpusNavigation";
import { navigateToCorpusEntryById, type NavigateHooks } from "./navigateToCorpusEntry";

type Props = {
  index: ReplayCorpusIndex;
  entryId: string | null;
  hooks: NavigateHooks;
};

export function CorpusBreadcrumbs({ index, entryId, hooks }: Props) {
  if (!entryId) return null;
  const entry = index.entries.find((e) => e.entry_id === entryId);
  if (!entry) return null;

  const ancestors = ancestorChain(entryId, index);
  const crumbs = [...ancestors, entryId];

  return (
    <nav className="mb-2 flex flex-wrap items-center gap-1 text-[10px] text-slate-400" aria-label="Corpus breadcrumbs">
      <span className="text-cyan-300">{index.corpus_id}</span>
      {entry.reviewer_category && (
        <>
          <span>/</span>
          <span>{REVIEWER_CATEGORY_LABELS[entry.reviewer_category] ?? entry.reviewer_category}</span>
        </>
      )}
      {entry.replay_family && (
        <>
          <span>/</span>
          <span>{entry.replay_family}</span>
        </>
      )}
      {crumbs.map((id, i) => (
        <span key={id} className="inline-flex items-center gap-1">
          <span>/</span>
          {i < crumbs.length - 1 ? (
            <button
              type="button"
              className="text-cyan-200 underline decoration-cyan-900/60 hover:text-cyan-100"
              onClick={() => void navigateToCorpusEntryById(id, hooks)}
            >
              {id.split("__").pop()}
            </button>
          ) : (
            <span className="text-slate-200">{entry.navigation_hint ?? id.split("__").pop()}</span>
          )}
        </span>
      ))}
    </nav>
  );
}
