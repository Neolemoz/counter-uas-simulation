import { readFileSync } from "node:fs";
import { join } from "node:path";
import { beforeEach, describe, expect, it, vi } from "vitest";
import {
  COHORT_STORAGE_KEY,
  exportCohortJson,
  getCohort,
  importCohortJson,
  listCohortIds,
  loadCohortMap,
  removeCohort,
  upsertCohort,
} from "./cohortIndexStore";

const REPO_ROOT = join(process.cwd(), "..", "..");
const FIXTURE = join(
  REPO_ROOT,
  "fixtures",
  "rt_experiments",
  "x2_cohort_index_example.json",
);

const memory: Record<string, string> = {};

vi.stubGlobal("localStorage", {
  getItem: (key: string) => memory[key] ?? null,
  setItem: (key: string, value: string) => {
    memory[key] = value;
  },
  removeItem: (key: string) => {
    delete memory[key];
  },
  clear: () => {
    for (const k of Object.keys(memory)) delete memory[k];
  },
});

describe("cohortIndexStore", () => {
  beforeEach(() => {
    for (const k of Object.keys(memory)) delete memory[k];
  });

  it("imports fixture and round-trips localStorage", () => {
    const text = readFileSync(FIXTURE, "utf-8");
    const cohort = importCohortJson(text);
    expect(cohort.cohort_id).toBe("cohort-2026-05-28-x2-example");
    expect(listCohortIds()).toContain(cohort.cohort_id);
    const map = loadCohortMap();
    expect(map[cohort.cohort_id]?.manifest_refs).toHaveLength(2);
    const exported = exportCohortJson(cohort.cohort_id);
    expect(JSON.parse(exported).cohort_id).toBe(cohort.cohort_id);
    expect(memory[COHORT_STORAGE_KEY]).toBeDefined();
  });

  it("removes cohort from map", () => {
    const text = readFileSync(FIXTURE, "utf-8");
    const cohort = importCohortJson(text);
    expect(removeCohort(cohort.cohort_id)).toBe(true);
    expect(getCohort(cohort.cohort_id)).toBeNull();
  });

  it("upsert replaces by cohort_id", () => {
    const base = importCohortJson(readFileSync(FIXTURE, "utf-8"));
    upsertCohort({ ...base, label: "Updated label" });
    expect(getCohort(base.cohort_id)?.label).toBe("Updated label");
  });
});
