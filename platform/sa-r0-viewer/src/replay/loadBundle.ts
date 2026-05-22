import { replaySaBundleSchema, type ReplaySaBundle } from "./bundleSchema";
import { normalizeBundleForSchema } from "./normalizeBundle";
import { loadScenarioCatalog, demoUrlFromPackId } from "./loadCatalog";

function parseBundleData(data: unknown): ReplaySaBundle {
  return replaySaBundleSchema.parse(normalizeBundleForSchema(data));
}

export async function loadBundleFromUrl(url: string): Promise<ReplaySaBundle> {
  const res = await fetch(url);
  if (!res.ok) {
    throw new Error(`Failed to load bundle: ${res.status} ${res.statusText}`);
  }
  const data: unknown = await res.json();
  return parseBundleData(data);
}

export function parseBundleJson(text: string): ReplaySaBundle {
  const data: unknown = JSON.parse(text);
  return parseBundleData(data);
}

export async function loadBundleFromFile(file: File): Promise<ReplaySaBundle> {
  const text = await file.text();
  if (file.name.endsWith(".zip")) {
    throw new Error("Zip bundles: extract index.json or use export-portable unpack first.");
  }
  return parseBundleJson(text);
}

export const DEMO_BUNDLE_URL = "/demo/index.json";

const DEMO_ALIASES: Record<string, string> = {
  ridge: "/demo/index.json",
  ridge_defense: "/demo/index.json",
  valley_ingress: "/demo/valley_ingress/index.json",
  multi_ridge: "/demo/multi_ridge/index.json",
  corridor_defense: "/demo/corridor_defense/index.json",
  saturation_ingress: "/demo/saturation_ingress/index.json",
  urban_masking: "/demo/urban_masking/index.json",
  delayed_detection: "/demo/delayed_detection/index.json",
  long_range_ingress: "/demo/long_range_ingress/index.json",
  valley_ingress_radar_shifted_north: "/demo/valley_ingress_radar_shifted_north/index.json",
  valley_ingress_extra_valley_sensor: "/demo/valley_ingress_extra_valley_sensor/index.json",
  valley_ingress_reduced_overlap_layout: "/demo/valley_ingress_reduced_overlap_layout/index.json",
  valley_ingress_delayed_interceptor_base: "/demo/valley_ingress_delayed_interceptor_base/index.json",
};

export function defaultDemoBundleUrl(): string {
  return DEMO_BUNDLE_URL;
}

export function demoAliasFromQuery(): string | null {
  const params = new URLSearchParams(window.location.search);
  return params.get("demo");
}

export function bundleUrlFromQuery(): string | null {
  const params = new URLSearchParams(window.location.search);
  return params.get("bundle");
}

/** Prefer ?bundle=, ?demo=pack_id (catalog), else public/demo/index.json. */
export async function resolveInitialBundleUrl(): Promise<string> {
  const fromQuery = bundleUrlFromQuery();
  if (fromQuery) return fromQuery;
  const demoAlias = demoAliasFromQuery();
  if (demoAlias) {
    try {
      const catalog = await loadScenarioCatalog();
      const fromCatalog = demoUrlFromPackId(catalog, demoAlias);
      if (fromCatalog) return fromCatalog;
    } catch {
      /* fall through to legacy aliases */
    }
    if (DEMO_ALIASES[demoAlias]) {
      return DEMO_ALIASES[demoAlias];
    }
  }
  try {
    const probe = await fetch(DEMO_BUNDLE_URL, { method: "HEAD" });
    if (probe.ok) return DEMO_BUNDLE_URL;
  } catch {
    /* offline or missing demo — still attempt default load */
  }
  return DEMO_BUNDLE_URL;
}
