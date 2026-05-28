import { safeParseCohortIndex } from "./cohortImportGuards";
import {
  experimentCohortIndexSchema,
  type ExperimentCohortIndex,
} from "./cohortSchema";

export const COHORT_STORAGE_KEY = "rt_experiment_cohort_index_v1";

export type CohortIndexMap = Record<string, ExperimentCohortIndex>;

export function loadCohortMap(): CohortIndexMap {
  if (typeof localStorage === "undefined") return {};
  const raw = localStorage.getItem(COHORT_STORAGE_KEY);
  if (!raw) return {};
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch {
    return {};
  }
  if (!parsed || typeof parsed !== "object" || Array.isArray(parsed)) {
    return {};
  }
  const out: CohortIndexMap = {};
  for (const [cohortId, value] of Object.entries(parsed as Record<string, unknown>)) {
    const result = experimentCohortIndexSchema.safeParse(value);
    if (result.success) {
      out[cohortId] = result.data;
    }
  }
  return out;
}

export function saveCohortMap(map: CohortIndexMap): void {
  if (typeof localStorage === "undefined") return;
  const validated: CohortIndexMap = {};
  for (const [cohortId, index] of Object.entries(map)) {
    validated[cohortId] = experimentCohortIndexSchema.parse(index);
  }
  localStorage.setItem(COHORT_STORAGE_KEY, JSON.stringify(validated, null, 2));
}

export function listCohortIds(): string[] {
  return Object.keys(loadCohortMap()).sort();
}

export function getCohort(cohortId: string): ExperimentCohortIndex | null {
  return loadCohortMap()[cohortId] ?? null;
}

export function upsertCohort(index: ExperimentCohortIndex): ExperimentCohortIndex {
  const parsed = experimentCohortIndexSchema.parse(index);
  const map = loadCohortMap();
  map[parsed.cohort_id] = parsed;
  saveCohortMap(map);
  return parsed;
}

export function removeCohort(cohortId: string): boolean {
  const map = loadCohortMap();
  if (!(cohortId in map)) return false;
  delete map[cohortId];
  saveCohortMap(map);
  return true;
}

export function importCohortJson(text: string): ExperimentCohortIndex {
  const parsed = safeParseCohortIndex(text);
  if (!parsed.ok) {
    throw new Error(parsed.error);
  }
  return upsertCohort(parsed.data);
}

export function exportCohortJson(cohortId: string): string {
  const cohort = getCohort(cohortId);
  if (!cohort) {
    throw new Error(`unknown cohort_id: ${cohortId}`);
  }
  return JSON.stringify(experimentCohortIndexSchema.parse(cohort), null, 2);
}
