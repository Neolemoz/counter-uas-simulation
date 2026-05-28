import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import {
  assertNoForbiddenManifestRefKeys,
  COHORT_GOVERNANCE_BANNER,
  experimentCohortIndexSchema,
} from "./cohortSchema";

const REPO_ROOT = join(process.cwd(), "..", "..");
const FIXTURE = join(
  REPO_ROOT,
  "fixtures",
  "rt_experiments",
  "x2_cohort_index_example.json",
);

describe("cohortSchema", () => {
  it("parses x2_cohort_index_example.json", () => {
    const raw = JSON.parse(readFileSync(FIXTURE, "utf-8"));
    const parsed = experimentCohortIndexSchema.parse(raw);
    expect(parsed.schema).toBe("rt_experiment_cohort_index_v1");
    expect(parsed.manifest_refs).toHaveLength(2);
    expect(parsed.governance_banner).toBe(COHORT_GOVERNANCE_BANNER);
  });

  it("rejects forbidden manifest_ref keys", () => {
    expect(() =>
      assertNoForbiddenManifestRefKeys({
        manifest_refs: [{ manifest_ref: "x", sa_corpus_ref: "bad" }],
      }),
    ).toThrow(/forbidden manifest_ref field/);
  });

  it("rejects unknown top-level keys", () => {
    expect(() =>
      experimentCohortIndexSchema.parse({
        schema: "rt_experiment_cohort_index_v1",
        cohort_id: "c1",
        label: "L",
        governance_banner: COHORT_GOVERNANCE_BANNER,
        manifest_refs: [
          {
            manifest_ref: "fixtures/rt_experiments/f5_metrics_golden/manifest.json",
            experiment_id: "e1",
            label: "one",
          },
        ],
        readiness_score: 1,
      }),
    ).toThrow();
  });
});
