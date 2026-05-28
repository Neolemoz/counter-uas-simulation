import { describe, expect, it } from "vitest";
import { COHORT_GOVERNANCE_BANNER } from "./cohortSchema";
import {
  assertCohortManifestRefAllowed,
  safeParseCohortIndex,
} from "./cohortImportGuards";

describe("cohortImportGuards", () => {
  it("blocks SA path manifest_ref", () => {
    expect(() =>
      assertCohortManifestRefAllowed("fixtures/sa_r0/replay/manifest.json"),
    ).toThrow(/blocked/);
  });

  it("parses valid cohort JSON", () => {
    const text = JSON.stringify({
      schema: "rt_experiment_cohort_index_v1",
      cohort_id: "cohort-test",
      label: "Test",
      governance_banner: COHORT_GOVERNANCE_BANNER,
      manifest_refs: [
        {
          manifest_ref: "fixtures/rt_experiments/f5_metrics_golden/manifest.json",
          experiment_id: "exp-1",
          label: "Golden",
        },
      ],
    });
    const result = safeParseCohortIndex(text);
    expect(result.ok).toBe(true);
    if (result.ok) {
      expect(result.data.cohort_id).toBe("cohort-test");
    }
  });

  it("rejects cohort with forbidden ref in manifest_refs", () => {
    const text = JSON.stringify({
      schema: "rt_experiment_cohort_index_v1",
      cohort_id: "cohort-bad",
      label: "Bad",
      governance_banner: COHORT_GOVERNANCE_BANNER,
      manifest_refs: [
        {
          manifest_ref: "fixtures/sa_r0/foo/manifest.json",
          experiment_id: "exp-1",
          label: "SA",
        },
      ],
    });
    const result = safeParseCohortIndex(text);
    expect(result.ok).toBe(false);
  });
});
