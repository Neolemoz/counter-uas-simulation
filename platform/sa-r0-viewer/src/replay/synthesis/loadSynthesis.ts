import {
  crossSweepSynthesisSchema,
  replayLinkageIndexSchema,
  type CrossSweepSynthesis,
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

export function relatedSweepIds(linkage: ReplayLinkageIndex, sweepId: string): string[] {
  const related = new Set<string>();
  for (const e of linkageEdgesForSweep(linkage, sweepId)) {
    if (e.source !== sweepId) related.add(e.source);
    if (e.target !== sweepId) related.add(e.target);
  }
  return [...related].sort();
}
