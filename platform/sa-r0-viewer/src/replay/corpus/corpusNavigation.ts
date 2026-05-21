import type { CorpusIndexEntry, ReplayCorpusIndex } from "../synthesis/synthesisSchema";

export type CorpusNavTarget =
  | { mode: "demo"; packId: string }
  | { mode: "sweep"; sweepId: string }
  | { mode: "presentation"; storyboardId: string }
  | { mode: "corpus_info" };

export const ENTRY_KIND_LABELS: Record<string, string> = {
  demo_bundle: "Demo bundle",
  sweep_family: "Sweep family",
  topology_experiment: "Topology experiment",
  presentation_deck: "Presentation deck",
  publication_packet: "Publication packet",
  synthesis_report: "Synthesis report",
  replay_export: "Replay export",
  research_bundle: "Research bundle",
  corpus_release: "Corpus release",
};

export const REVIEWER_CATEGORY_LABELS: Record<string, string> = {
  topology_lab: "Topology lab",
  sweep_experiment: "Sweep experiments",
  presentation: "Presentations",
  synthesis: "Synthesis",
  export: "Exports",
  corpus_ops: "Corpus operations",
};

export function entrySlug(entryId: string): string {
  const i = entryId.indexOf("__");
  return i >= 0 ? entryId.slice(i + 2) : entryId;
}

export function resolveEntryTarget(entry: CorpusIndexEntry): CorpusNavTarget | null {
  const replay = entry.replay_scope as { pack_id?: string } | undefined;
  const sweep = entry.sweep_scope as { sweep_id?: string; sweep_ids?: string[] } | undefined;
  const pres = entry.presentation_scope as { storyboard_id?: string } | undefined;

  switch (entry.entry_kind) {
    case "demo_bundle":
      if (replay?.pack_id) return { mode: "demo", packId: replay.pack_id };
      return { mode: "demo", packId: entrySlug(entry.entry_id) };
    case "topology_experiment":
      return { mode: "demo", packId: replay?.pack_id ?? entrySlug(entry.entry_id) };
    case "sweep_family":
      if (sweep?.sweep_id) return { mode: "sweep", sweepId: sweep.sweep_id };
      return { mode: "sweep", sweepId: entrySlug(entry.entry_id) };
    case "presentation_deck":
      if (pres?.storyboard_id) return { mode: "presentation", storyboardId: pres.storyboard_id };
      return { mode: "presentation", storyboardId: entrySlug(entry.entry_id) };
    case "publication_packet":
    case "replay_export": {
      const sid = sweep?.sweep_id;
      if (sid) return { mode: "sweep", sweepId: sid };
      const slug = entrySlug(entry.entry_id);
      const base = slug.replace(/_publication$/, "").replace(/_(compare|presentation|review)$/, "");
      return { mode: "sweep", sweepId: base };
    }
    case "synthesis_report": {
      const ids = sweep?.sweep_ids;
      if (ids?.length) return { mode: "sweep", sweepId: ids[0]! };
      return { mode: "sweep", sweepId: "valley_sensor_sweep" };
    }
    case "research_bundle":
      return { mode: "corpus_info" };
    default:
      return { mode: "corpus_info" };
  }
}

export function parentsOf(entryId: string, index: ReplayCorpusIndex): string[] {
  const entry = index.entries.find((e) => e.entry_id === entryId);
  return [...(entry?.lineage_parent_ids ?? [])].sort();
}

export function childrenOf(entryId: string, index: ReplayCorpusIndex): string[] {
  const fromEdges = (index.lineage_edges ?? [])
    .filter((e) => e.parent_entry_id === entryId)
    .map((e) => e.child_entry_id);
  const fromEntries = index.entries
    .filter((e) => (e.lineage_parent_ids ?? []).includes(entryId))
    .map((e) => e.entry_id);
  return [...new Set([...fromEdges, ...fromEntries])].sort();
}

export function ancestorChain(
  entryId: string,
  index: ReplayCorpusIndex,
  maxDepth = 8,
): string[] {
  const chain: string[] = [];
  let current = entryId;
  const seen = new Set<string>();
  for (let d = 0; d < maxDepth; d++) {
    const parents = parentsOf(current, index);
    if (!parents.length) break;
    const parent = parents[0]!;
    if (seen.has(parent)) break;
    seen.add(parent);
    chain.unshift(parent);
    current = parent;
  }
  return chain;
}

export function siblingsOf(entryId: string, index: ReplayCorpusIndex): string[] {
  const parents = parentsOf(entryId, index);
  if (!parents.length) return [];
  const parent = parents[0]!;
  return childrenOf(parent, index).filter((id) => id !== entryId);
}

export function findingsForEntry<T extends { entry_id?: string }>(
  findings: T[],
  entryId: string,
): T[] {
  return findings.filter((f) => f.entry_id === entryId);
}

export type CorpusGroupMode = "family" | "chronology" | "kind";

export function groupEntries(
  entries: CorpusIndexEntry[],
  mode: CorpusGroupMode,
): Map<string, CorpusIndexEntry[]> {
  const map = new Map<string, CorpusIndexEntry[]>();
  for (const e of entries) {
    let key: string;
    if (mode === "family") key = e.replay_family ?? e.entry_kind;
    else if (mode === "chronology")
      key = e.replay_chronology_descriptor ?? e.chronology_group ?? e.reviewer_category ?? e.entry_kind;
    else key = e.entry_kind;
    if (!map.has(key)) map.set(key, []);
    map.get(key)!.push(e);
  }
  for (const list of map.values()) {
    list.sort((a, b) => a.entry_id.localeCompare(b.entry_id));
  }
  return new Map([...map.entries()].sort(([a], [b]) => a.localeCompare(b)));
}

export const DECLUTTER_COLLAPSED_KINDS = new Set(["demo_bundle", "replay_export"]);
