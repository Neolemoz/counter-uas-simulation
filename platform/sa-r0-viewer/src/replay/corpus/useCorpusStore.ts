import { create } from "zustand";
import type { CorpusGroupMode } from "./corpusNavigation";

type CorpusState = {
  selectedEntryId: string | null;
  browserOpen: boolean;
  showAllKinds: boolean;
  groupMode: CorpusGroupMode;
  activeKindFilter: string | null;
  activeCategoryFilter: string | null;
  activeTags: string[];
  evolutionTags: string[];
  searchQuery: string;
  highlightChronologyTier: string | null;
  setSelectedEntryId: (id: string | null) => void;
  setBrowserOpen: (open: boolean) => void;
  setShowAllKinds: (v: boolean) => void;
  setGroupMode: (mode: CorpusGroupMode) => void;
  setActiveKindFilter: (kind: string | null) => void;
  setActiveCategoryFilter: (cat: string | null) => void;
  toggleTag: (tag: string) => void;
  toggleEvolutionTag: (tag: string) => void;
  setSearchQuery: (q: string) => void;
  setHighlightChronologyTier: (tier: string | null) => void;
};

export const useCorpusStore = create<CorpusState>((set) => ({
  selectedEntryId: null,
  browserOpen: false,
  showAllKinds: false,
  groupMode: "family",
  activeKindFilter: null,
  activeCategoryFilter: null,
  activeTags: [],
  evolutionTags: [],
  searchQuery: "",
  highlightChronologyTier: null,
  setSelectedEntryId: (selectedEntryId) => set({ selectedEntryId }),
  setBrowserOpen: (browserOpen) => set({ browserOpen }),
  setShowAllKinds: (showAllKinds) => set({ showAllKinds }),
  setGroupMode: (groupMode) => set({ groupMode }),
  setActiveKindFilter: (activeKindFilter) => set({ activeKindFilter }),
  setActiveCategoryFilter: (activeCategoryFilter) => set({ activeCategoryFilter }),
  toggleTag: (tag) =>
    set((s) => ({
      activeTags: s.activeTags.includes(tag)
        ? s.activeTags.filter((t) => t !== tag)
        : [...s.activeTags, tag],
    })),
  toggleEvolutionTag: (tag) =>
    set((s) => ({
      evolutionTags: s.evolutionTags.includes(tag)
        ? s.evolutionTags.filter((t) => t !== tag)
        : [...s.evolutionTags, tag],
    })),
  setSearchQuery: (searchQuery) => set({ searchQuery }),
  setHighlightChronologyTier: (highlightChronologyTier) => set({ highlightChronologyTier }),
}));

export function readCorpusChronologyFromUrl(): string | null {
  return new URLSearchParams(window.location.search).get("corpus_chronology");
}

export function setCorpusEntryUrlParam(entryId: string | null): void {
  const url = new URL(window.location.href);
  if (entryId) url.searchParams.set("corpus_entry", entryId);
  else url.searchParams.delete("corpus_entry");
  window.history.replaceState({}, "", url.toString());
}

export function readCorpusEntryFromUrl(): string | null {
  return new URLSearchParams(window.location.search).get("corpus_entry");
}
