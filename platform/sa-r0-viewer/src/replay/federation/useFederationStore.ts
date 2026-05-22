import { create } from "zustand";

type FederationState = {
  federationId: string | null;
  corpusGroupId: string | null;
  highlightedLineageRef: string | null;
  setFederationId: (id: string | null) => void;
  setCorpusGroupId: (id: string | null) => void;
  setHighlightedLineageRef: (ref: string | null) => void;
};

export const useFederationStore = create<FederationState>((set) => ({
  federationId: null,
  corpusGroupId: null,
  highlightedLineageRef: null,
  setFederationId: (federationId) => set({ federationId }),
  setCorpusGroupId: (corpusGroupId) => set({ corpusGroupId }),
  setHighlightedLineageRef: (highlightedLineageRef) => set({ highlightedLineageRef }),
}));

export function readFederationFromUrl(): {
  federationId: string | null;
  corpusGroupId: string | null;
  federationLineageRef: string | null;
} {
  if (typeof window === "undefined") {
    return { federationId: null, corpusGroupId: null, federationLineageRef: null };
  }
  const params = new URLSearchParams(window.location.search);
  return {
    federationId: params.get("federation_id"),
    corpusGroupId: params.get("corpus_group_id"),
    federationLineageRef: params.get("federation_lineage_ref"),
  };
}

export function writeFederationToUrl(
  federationId: string | null,
  corpusGroupId: string | null,
  federationLineageRef: string | null,
): void {
  if (typeof window === "undefined") return;
  const url = new URL(window.location.href);
  if (federationId) url.searchParams.set("federation_id", federationId);
  else url.searchParams.delete("federation_id");
  if (corpusGroupId) url.searchParams.set("corpus_group_id", corpusGroupId);
  else url.searchParams.delete("corpus_group_id");
  if (federationLineageRef) url.searchParams.set("federation_lineage_ref", federationLineageRef);
  else url.searchParams.delete("federation_lineage_ref");
  window.history.replaceState({}, "", url.toString());
}
