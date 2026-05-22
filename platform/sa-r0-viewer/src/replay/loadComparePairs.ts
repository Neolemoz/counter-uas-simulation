import { comparePairsSchema, COMPARE_PAIRS_URL, type ComparePairsManifest } from "./catalogSchema";

let cached: ComparePairsManifest | null = null;

export async function loadComparePairs(): Promise<ComparePairsManifest> {
  if (cached) return cached;
  const res = await fetch(COMPARE_PAIRS_URL);
  if (!res.ok) throw new Error(`Failed to load compare pairs: ${res.status}`);
  const data: unknown = await res.json();
  cached = comparePairsSchema.parse(data);
  return cached;
}

export function pairById(manifest: ComparePairsManifest, pairId: string) {
  return manifest.pairs.find((p) => p.pair_id === pairId) ?? null;
}
