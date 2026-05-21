import {
  replayCorpusEvolutionManifestSchema,
  replayCorpusEvolutionSummarySchema,
  type ReplayCorpusEvolutionManifest,
  type ReplayCorpusEvolutionSummary,
} from "../synthesis/synthesisSchema";

export const EVOLUTION_MANIFEST_URL = "/demo/synthesis/replay_corpus_evolution_manifest_v1.json";
export const EVOLUTION_SUMMARY_URL = "/demo/synthesis/replay_corpus_evolution_summary_v1.json";

export async function loadCorpusEvolutionManifest(): Promise<ReplayCorpusEvolutionManifest> {
  const res = await fetch(EVOLUTION_MANIFEST_URL);
  if (!res.ok) throw new Error(`Failed to load evolution manifest: ${res.status}`);
  const data: unknown = await res.json();
  return replayCorpusEvolutionManifestSchema.parse(data);
}

export async function loadCorpusEvolutionSummary(): Promise<ReplayCorpusEvolutionSummary> {
  const res = await fetch(EVOLUTION_SUMMARY_URL);
  if (!res.ok) throw new Error(`Failed to load evolution summary: ${res.status}`);
  const data: unknown = await res.json();
  return replayCorpusEvolutionSummarySchema.parse(data);
}
