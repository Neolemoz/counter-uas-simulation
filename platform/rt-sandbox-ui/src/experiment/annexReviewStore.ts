import type { ExperimentManifest } from "./experimentSchema";
import { safeParseAnnex, safeParseAnnexBundle } from "./experimentImportGuards";
import {
  parseTacticalAnnexJson,
  rtTacticalCaptureAnnexSchema,
  type ExperimentAnnexBundle,
  type TacticalCaptureAnnex,
} from "./tacticalAnnexSchema";

const CACHE_KEY = "rt_experiment_annex_cache_v1";

export type AnnexCache = Record<string, TacticalCaptureAnnex>;

export type AnnexBundleImportResult = {
  imported: number;
  skipped: number;
  errors: string[];
};

function readCache(): AnnexCache {
  if (typeof localStorage === "undefined") return {};
  const raw = localStorage.getItem(CACHE_KEY);
  if (!raw) return {};
  let parsed: Record<string, unknown>;
  try {
    parsed = JSON.parse(raw) as Record<string, unknown>;
  } catch {
    return {};
  }
  const out: AnnexCache = {};
  for (const [runId, value] of Object.entries(parsed)) {
    const result = rtTacticalCaptureAnnexSchema.safeParse(value);
    if (result.success) {
      out[runId] = result.data;
    }
  }
  return out;
}

function writeCache(cache: AnnexCache): void {
  if (typeof localStorage === "undefined") return;
  localStorage.setItem(CACHE_KEY, JSON.stringify(cache, null, 2));
}

export function getAnnexForRun(runId: string): TacticalCaptureAnnex | null {
  return readCache()[runId] ?? null;
}

export function getAnnexCache(): AnnexCache {
  return readCache();
}

export function importAnnexForRun(runId: string, text: string): TacticalCaptureAnnex {
  const parsed = safeParseAnnex(text);
  if (!parsed.ok) {
    throw new Error(parsed.error);
  }
  const cache = readCache();
  cache[runId] = parsed.data;
  writeCache(cache);
  return parsed.data;
}

export function clearAnnexForRun(runId: string): void {
  const cache = readCache();
  delete cache[runId];
  writeCache(cache);
}

export function pruneAnnexCacheForManifest(manifest: ExperimentManifest): number {
  const valid = new Set(manifest.runs.map((r) => r.run_id));
  const cache = readCache();
  let removed = 0;
  for (const runId of Object.keys(cache)) {
    if (!valid.has(runId)) {
      delete cache[runId];
      removed += 1;
    }
  }
  if (removed > 0) {
    writeCache(cache);
  }
  return removed;
}

export function importAnnexBundle(text: string): number {
  return importAnnexBundleDetailed(text).imported;
}

export function importAnnexBundleDetailed(text: string): AnnexBundleImportResult {
  let raw: unknown;
  try {
    raw = JSON.parse(text);
  } catch {
    return { imported: 0, skipped: 0, errors: ["invalid JSON"] };
  }
  if (
    !raw ||
    typeof raw !== "object" ||
    (raw as { schema?: string }).schema !== "rt_experiment_annex_bundle_v1" ||
    !Array.isArray((raw as { entries?: unknown }).entries)
  ) {
    const bundleResult = safeParseAnnexBundle(text);
    return {
      imported: 0,
      skipped: 0,
      errors: [bundleResult.ok ? "invalid annex bundle" : bundleResult.error],
    };
  }
  const entries = (raw as { entries: unknown[] }).entries;
  const cache = readCache();
  let imported = 0;
  let skipped = 0;
  const errors: string[] = [];
  for (const entry of entries) {
    if (!entry || typeof entry !== "object") {
      skipped += 1;
      errors.push("entry: invalid shape");
      continue;
    }
    const runId = (entry as { run_id?: string }).run_id;
    if (!runId) {
      skipped += 1;
      errors.push("entry: missing run_id");
      continue;
    }
    const annexResult = rtTacticalCaptureAnnexSchema.safeParse(
      (entry as { annex?: unknown }).annex,
    );
    if (!annexResult.success) {
      skipped += 1;
      errors.push(`${runId}: invalid annex`);
      continue;
    }
    cache[runId] = annexResult.data;
    imported += 1;
  }
  writeCache(cache);
  return { imported, skipped, errors };
}

export function exportAnnexBundle(manifest: ExperimentManifest): string {
  const cache = readCache();
  const entries = manifest.runs
    .filter((r) => cache[r.run_id])
    .map((r) => ({
      run_id: r.run_id,
      capture_candidate_id: r.capture_candidate_id ?? null,
      annex: cache[r.run_id],
    }));
  const bundle: ExperimentAnnexBundle = {
    schema: "rt_experiment_annex_bundle_v1",
    experiment_id: manifest.experiment_id,
    entries,
  };
  return JSON.stringify(bundle, null, 2);
}

export function findRunIdForAnnex(
  manifest: ExperimentManifest,
  annex: TacticalCaptureAnnex,
): string | null {
  const capId = annex.capture_candidate_id;
  if (capId) {
    const match = manifest.runs.find((r) => r.capture_candidate_id === capId);
    if (match) return match.run_id;
  }
  return null;
}

/** @deprecated use safeParseAnnex via importAnnexForRun */
export { parseTacticalAnnexJson };
