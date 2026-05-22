import { scenarioCatalogSchema, type ScenarioCatalog } from "./catalogSchema";

let cachedCatalog: ScenarioCatalog | null = null;

export async function loadScenarioCatalog(): Promise<ScenarioCatalog> {
  if (cachedCatalog) return cachedCatalog;
  const res = await fetch("/demo/catalog.json");
  if (!res.ok) {
    throw new Error(`Failed to load scenario catalog: ${res.status}`);
  }
  const data: unknown = await res.json();
  cachedCatalog = scenarioCatalogSchema.parse(data);
  return cachedCatalog;
}

export function demoUrlFromPackId(catalog: ScenarioCatalog, packId: string): string | null {
  const entry = catalog.packs.find((p) => p.pack_id === packId);
  return entry?.demo_bundle_url ?? null;
}

export function allCatalogTags(catalog: ScenarioCatalog): string[] {
  const tags = new Set<string>();
  for (const pack of catalog.packs) {
    for (const t of pack.topology_tags) tags.add(t);
    for (const t of pack.replay_tags ?? []) tags.add(t);
  }
  return [...tags].sort();
}
