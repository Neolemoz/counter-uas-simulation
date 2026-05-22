import {
  crossSweepSynthesisSchema,
  replayCorpusIndexSchema,
  replayLinkageIndexSchema,
  type CrossSweepSynthesis,
  type ReplayCorpusIndex,
  type ReplayLinkageIndex,
} from "./synthesisSchema";

export async function loadCrossSweepSynthesis(): Promise<CrossSweepSynthesis> {
  const res = await fetch("/demo/synthesis/cross_sweep_synthesis_v1.json");
  if (!res.ok) throw new Error(`Failed to load synthesis: ${res.status}`);
  const data: unknown = await res.json();
  return crossSweepSynthesisSchema.parse(data);
}

export async function loadReplayLinkageIndex(): Promise<ReplayLinkageIndex> {
  const res = await fetch("/demo/synthesis/replay_linkage_index_v1.json");
  if (!res.ok) throw new Error(`Failed to load linkage index: ${res.status}`);
  const data: unknown = await res.json();
  return replayLinkageIndexSchema.parse(data);
}

export function linkageEdgesForSweep(
  linkage: ReplayLinkageIndex,
  sweepId: string,
): ReplayLinkageIndex["edges"] {
  return linkage.edges.filter((e) => e.source === sweepId || e.target === sweepId);
}

export async function loadReplayCorpusIndex(): Promise<ReplayCorpusIndex> {
  const res = await fetch("/demo/synthesis/replay_corpus_index_v1.json");
  if (!res.ok) throw new Error(`Failed to load corpus index: ${res.status}`);
  const data: unknown = await res.json();
  return replayCorpusIndexSchema.parse(data);
}

export function corpusEntryForSweep(
  index: ReplayCorpusIndex,
  sweepId: string,
): ReplayCorpusIndex["entries"][number] | undefined {
  const eid = `sweep_family__${sweepId}`;
  return index.entries.find((e) => e.entry_id === eid);
}

export function relatedSweepIds(linkage: ReplayLinkageIndex, sweepId: string): string[] {
  const related = new Set<string>();
  for (const e of linkageEdgesForSweep(linkage, sweepId)) {
    if (e.source !== sweepId) related.add(e.source);
    if (e.target !== sweepId) related.add(e.target);
  }
  return [...related].sort();
}
