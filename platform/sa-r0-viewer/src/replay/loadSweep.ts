import {
  replayMcSweepSchema,
  sweepsIndexSchema,
  type ReplayMcSweep,
  type SweepsIndex,
} from "./sweepSchema";

export async function loadSweepsIndex(): Promise<SweepsIndex> {
  const res = await fetch("/demo/sweeps_index.json");
  if (!res.ok) throw new Error(`Failed to load sweeps index: ${res.status}`);
  const data: unknown = await res.json();
  return sweepsIndexSchema.parse(data);
}

export async function loadSweepManifest(sweepId: string): Promise<ReplayMcSweep> {
  const res = await fetch(`/demo/sweeps/${sweepId}/sweep.json`);
  if (!res.ok) throw new Error(`Failed to load sweep ${sweepId}: ${res.status}`);
  const data: unknown = await res.json();
  return replayMcSweepSchema.parse(data);
}

export function sweepEntryById(index: SweepsIndex, sweepId: string) {
  return index.sweeps.find((s) => s.sweep_id === sweepId);
}
