import { useEffect, useMemo, useState } from "react";
import { loadReplayCorpusIndex } from "../synthesis/loadSynthesis";
import type { CorpusIndexEntry, ReplayCorpusIndex } from "../synthesis/synthesisSchema";
import {
  DECLUTTER_COLLAPSED_KINDS,
  ENTRY_KIND_LABELS,
  REVIEWER_CATEGORY_LABELS,
  groupEntries,
} from "./corpusNavigation";
import { navigateToCorpusEntry, type NavigateHooks } from "./navigateToCorpusEntry";
import { CorpusBreadcrumbs } from "./CorpusBreadcrumbs";
import { CorpusReleaseBrowser } from "./CorpusReleaseBrowser";
import { useCorpusStore } from "./useCorpusStore";

type Props = {
  hooks: NavigateHooks;
};

function matchesEntry(
  e: CorpusIndexEntry,
  opts: {
    kind: string | null;
    category: string | null;
    tags: string[];
    evolutionTags: string[];
    query: string;
    showAllKinds: boolean;
  },
): boolean {
  if (!opts.showAllKinds && DECLUTTER_COLLAPSED_KINDS.has(e.entry_kind)) return false;
  if (opts.kind && e.entry_kind !== opts.kind) return false;
  if (opts.category && e.reviewer_category !== opts.category) return false;
  if (opts.tags.length && !opts.tags.every((t) => (e.navigation_tags ?? []).includes(t)))
    return false;
  if (
    opts.evolutionTags.length &&
    !opts.evolutionTags.every((t) => (e.evolution_tags ?? []).includes(t))
  )
    return false;
  if (opts.query) {
    const q = opts.query.toLowerCase();
    const hay = [
      e.entry_id,
      e.navigation_hint ?? "",
      e.primary_artifact_path,
      e.replay_family ?? "",
    ]
      .join(" ")
      .toLowerCase();
    if (!hay.includes(q)) return false;
  }
  return true;
}

export function CorpusBrowserPanel({ hooks }: Props) {
  const [index, setIndex] = useState<ReplayCorpusIndex | null>(null);
  const [loadErr, setLoadErr] = useState<string | null>(null);
  const browserOpen = useCorpusStore((s) => s.browserOpen);
  const setBrowserOpen = useCorpusStore((s) => s.setBrowserOpen);
  const selectedEntryId = useCorpusStore((s) => s.selectedEntryId);
  const showAllKinds = useCorpusStore((s) => s.showAllKinds);
  const setShowAllKinds = useCorpusStore((s) => s.setShowAllKinds);
  const groupMode = useCorpusStore((s) => s.groupMode);
  const setGroupMode = useCorpusStore((s) => s.setGroupMode);
  const activeKindFilter = useCorpusStore((s) => s.activeKindFilter);
  const setActiveKindFilter = useCorpusStore((s) => s.setActiveKindFilter);
  const activeCategoryFilter = useCorpusStore((s) => s.activeCategoryFilter);
  const setActiveCategoryFilter = useCorpusStore((s) => s.setActiveCategoryFilter);
  const activeTags = useCorpusStore((s) => s.activeTags);
  const toggleTag = useCorpusStore((s) => s.toggleTag);
  const evolutionTags = useCorpusStore((s) => s.evolutionTags);
  const toggleEvolutionTag = useCorpusStore((s) => s.toggleEvolutionTag);
  const searchQuery = useCorpusStore((s) => s.searchQuery);
  const setSearchQuery = useCorpusStore((s) => s.setSearchQuery);

  useEffect(() => {
    loadReplayCorpusIndex()
      .then((idx) => {
        setIndex(idx);
        setLoadErr(null);
      })
      .catch((e: unknown) => setLoadErr(String(e)));
  }, []);

  const tagPool = useMemo(() => {
    if (!index) return [];
    const tags = new Set<string>();
    for (const e of index.entries) {
      for (const t of e.navigation_tags ?? []) tags.add(t);
    }
    return [...tags].sort();
  }, [index]);

  const evolutionTagPool = useMemo(() => {
    if (!index) return [];
    const tags = new Set<string>();
    for (const e of index.entries) {
      for (const t of e.evolution_tags ?? []) tags.add(t);
    }
    return [...tags].sort();
  }, [index]);

  const kinds = useMemo(() => {
    if (!index) return [];
    return [...new Set(index.entries.map((e) => e.entry_kind))].sort();
  }, [index]);

  const categories = useMemo(() => {
    if (!index) return [];
    return [...new Set(index.entries.map((e) => e.reviewer_category).filter(Boolean))].sort() as string[];
  }, [index]);

  const filtered = useMemo(() => {
    if (!index) return [];
    return index.entries.filter((e) =>
      matchesEntry(e, {
        kind: activeKindFilter,
        category: activeCategoryFilter,
        tags: activeTags,
        evolutionTags,
        query: searchQuery.trim(),
        showAllKinds,
      }),
    );
  }, [
    index,
    activeKindFilter,
    activeCategoryFilter,
    activeTags,
    evolutionTags,
    searchQuery,
    showAllKinds,
  ]);

  const grouped = useMemo(() => groupEntries(filtered, groupMode), [filtered, groupMode]);

  if (loadErr) {
    return (
      <p className="mb-2 text-xs text-red-400">Corpus index: {loadErr}</p>
    );
  }

  if (!index) {
    return (
      <p className="mb-2 text-xs text-slate-500">Loading corpus inventory…</p>
    );
  }

  return (
    <section className="mb-3 rounded border border-cyan-900/50 bg-cyan-950/15 p-3 text-sm">
      <button
        type="button"
        className="mb-2 flex w-full items-center justify-between font-semibold text-cyan-100"
        onClick={() => setBrowserOpen(!browserOpen)}
      >
        <span>Corpus inventory ({index.entries.length} entries)</span>
        <span className="text-xs text-slate-400">{browserOpen ? "−" : "+"}</span>
      </button>
      <p className="mb-2 text-[10px] text-slate-500">
        {index.governance?.notice ??
          "Deterministic corpus browser — replay-local navigation only."}
      </p>
      {browserOpen && (
        <>
          <CorpusBreadcrumbs index={index} entryId={selectedEntryId} hooks={hooks} />
          <label className="mb-2 block text-xs text-slate-400">
            Search
            <input
              type="search"
              className="mt-1 w-full rounded border border-slate-600 bg-slate-800 px-2 py-1 text-slate-200"
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              placeholder="entry id, hint, path…"
            />
          </label>
          <div className="mb-2 flex flex-wrap gap-1">
            <button
              type="button"
              className={`rounded px-2 py-0.5 text-[10px] ${!activeKindFilter ? "bg-cyan-900/60 text-cyan-100" : "bg-slate-800 text-slate-400"}`}
              onClick={() => setActiveKindFilter(null)}
            >
              All kinds
            </button>
            {kinds.map((k) => (
              <button
                key={k}
                type="button"
                className={`rounded px-2 py-0.5 text-[10px] ${activeKindFilter === k ? "bg-cyan-900/60 text-cyan-100" : "bg-slate-800 text-slate-400"}`}
                onClick={() => setActiveKindFilter(activeKindFilter === k ? null : k)}
              >
                {ENTRY_KIND_LABELS[k] ?? k}
              </button>
            ))}
          </div>
          <div className="mb-2 flex flex-wrap gap-1">
            <button
              type="button"
              className={`rounded px-2 py-0.5 text-[10px] ${!activeCategoryFilter ? "bg-violet-900/40 text-violet-100" : "bg-slate-800 text-slate-400"}`}
              onClick={() => setActiveCategoryFilter(null)}
            >
              All categories
            </button>
            {categories.map((c) => (
              <button
                key={c}
                type="button"
                className={`rounded px-2 py-0.5 text-[10px] ${activeCategoryFilter === c ? "bg-violet-900/40 text-violet-100" : "bg-slate-800 text-slate-400"}`}
                onClick={() =>
                  setActiveCategoryFilter(activeCategoryFilter === c ? null : c)
                }
              >
                {REVIEWER_CATEGORY_LABELS[c] ?? c}
              </button>
            ))}
          </div>
          {tagPool.length > 0 && (
            <div className="mb-2 flex flex-wrap gap-1">
              {tagPool.map((t) => (
                <button
                  key={t}
                  type="button"
                  className={`rounded px-2 py-0.5 text-[10px] ${activeTags.includes(t) ? "bg-slate-600 text-slate-100" : "bg-slate-800 text-slate-500"}`}
                  onClick={() => toggleTag(t)}
                >
                  {t}
                </button>
              ))}
            </div>
          )}
          {evolutionTagPool.length > 0 && (
            <div className="mb-2 flex flex-wrap gap-1">
              <span className="w-full text-[10px] text-violet-400/80">Evolution tags</span>
              {evolutionTagPool.map((t) => (
                <button
                  key={t}
                  type="button"
                  className={`rounded px-2 py-0.5 text-[10px] ${evolutionTags.includes(t) ? "bg-violet-900/60 text-violet-100" : "bg-slate-800 text-slate-500"}`}
                  onClick={() => toggleEvolutionTag(t)}
                >
                  {t}
                </button>
              ))}
            </div>
          )}
          <div className="mb-2 flex flex-wrap items-center gap-2 text-[10px] text-slate-400">
            <label className="inline-flex items-center gap-1">
              <input
                type="checkbox"
                checked={showAllKinds}
                onChange={(e) => setShowAllKinds(e.target.checked)}
              />
              Show demo bundles & exports
            </label>
            <span>Group:</span>
            {(["family", "chronology", "kind"] as const).map((m) => (
              <button
                key={m}
                type="button"
                className={`rounded px-2 py-0.5 ${groupMode === m ? "bg-slate-600 text-slate-100" : "bg-slate-800"}`}
                onClick={() => setGroupMode(m)}
              >
                {m}
              </button>
            ))}
          </div>
          <div className="max-h-48 overflow-y-auto rounded border border-slate-800 bg-slate-950/50 p-2">
            {[...grouped.entries()].map(([groupKey, items]) => (
              <div key={groupKey} className="mb-2">
                <p className="text-[10px] font-medium uppercase text-slate-500">{groupKey}</p>
                <ul className="space-y-1">
                  {items.map((e) => (
                    <li key={e.entry_id}>
                      <button
                        type="button"
                        className={`w-full text-left text-xs ${e.entry_id === selectedEntryId ? "text-cyan-200" : "text-slate-300 hover:text-cyan-100"}`}
                        onClick={() => void navigateToCorpusEntry(e, hooks)}
                      >
                        <span className="font-medium">
                          {e.navigation_hint ?? e.entry_id.split("__").pop()}
                        </span>
                        <span className="ml-1 text-[10px] text-slate-500">
                          ({e.entry_kind})
                        </span>
                      </button>
                    </li>
                  ))}
                </ul>
              </div>
            ))}
            {filtered.length === 0 && (
              <p className="text-xs text-slate-500">No entries match filters.</p>
            )}
          </div>
          <CorpusReleaseBrowser hooks={hooks} selectedEntryId={selectedEntryId} />
        </>
      )}
    </section>
  );
}
