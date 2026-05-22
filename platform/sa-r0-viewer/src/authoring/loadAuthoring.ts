import {
  authoringMirrorIndexSchema,
  authoringIntegrityReportSchema,
  scenarioAuthoringManifestSchema,
  AUTHORING_INDEX_URL,
  AUTHORING_INTEGRITY_URL,
  authoringManifestUrl,
  type ScenarioAuthoringManifest,
  type AuthoringIntegrityReport,
} from "./authoringSchema";

export async function loadAuthoringManifest(
  packId: string,
): Promise<ScenarioAuthoringManifest | null> {
  const res = await fetch(authoringManifestUrl(packId));
  if (!res.ok) return null;
  return scenarioAuthoringManifestSchema.parse(await res.json());
}

export async function loadAuthoringIndex(): Promise<{ pack_id: string; url: string }[]> {
  const res = await fetch(AUTHORING_INDEX_URL);
  if (!res.ok) return [];
  const data = authoringMirrorIndexSchema.parse(await res.json());
  return data.entries;
}

export async function loadAuthoringIntegrityReport(): Promise<AuthoringIntegrityReport | null> {
  const res = await fetch(AUTHORING_INTEGRITY_URL);
  if (!res.ok) return null;
  return authoringIntegrityReportSchema.parse(await res.json());
}

/** Walk parent_pack_id chain via mirrored manifests (read-only). */
export async function loadAuthoringAncestorChain(
  packId: string,
  maxDepth = 8,
): Promise<{ pack_id: string; parent_pack_id: string | null; promotion_status: string }[]> {
  const chain: { pack_id: string; parent_pack_id: string | null; promotion_status: string }[] = [];
  let current: string | null = packId;
  const seen = new Set<string>();

  while (current && chain.length < maxDepth && !seen.has(current)) {
    seen.add(current);
    const manifest = await loadAuthoringManifest(current);
    const parent = manifest?.parent_pack_id ?? null;
    chain.push({
      pack_id: current,
      parent_pack_id: parent,
      promotion_status: manifest?.promotion_status ?? "none",
    });
    if (!parent || parent === current) break;
    current = parent;
  }
  return chain;
}

export function isFingerprintStale(
  manifest: ScenarioAuthoringManifest,
  mirrorCheckedAt?: string | null,
): boolean {
  if (!manifest.validation_pack_fingerprint) return true;
  if (mirrorCheckedAt && manifest.updated_at) {
    return manifest.updated_at > mirrorCheckedAt;
  }
  return false;
}
