import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { experimentCohortIndexSchema } from "./cohortSchema";
import { buildMultiManifestDiff, buildManifestSummaryChips } from "./multiManifestDiff";
import type { ExperimentManifest } from "./experimentSchema";

const REPO_ROOT = join(import.meta.dirname, "../../../..");
const cohortFixture = JSON.parse(
  readFileSync(
    join(REPO_ROOT, "fixtures/rt_experiments/x2_cohort_index_example.json"),
    "utf-8",
  ),
);
const cohort = experimentCohortIndexSchema.parse(cohortFixture);

describe("multiManifestDiff", () => {
  it("builds metadata rows for two cohort manifests", () => {
    const primary = cohort.manifest_refs[0]!.manifest_ref;
    const secondary = cohort.manifest_refs[1]!.manifest_ref;
    const rows = buildMultiManifestDiff({
      cohort,
      primaryManifestRef: primary,
      secondaryManifestRef: secondary,
    });
    expect(rows.length).toBeGreaterThanOrEqual(5);
    const expRow = rows.find((r) => r.field === "experiment_id");
    expect(expRow?.primary).toBe(cohort.manifest_refs[0]!.experiment_id);
    expect(expRow?.secondary).toBe(cohort.manifest_refs[1]!.experiment_id);
    expect(expRow?.compare_status).toBe("divergent");
    const runsRow = rows.find((r) => r.field === "runs.length");
    expect(runsRow?.primary).toBe("4");
    expect(runsRow?.secondary).toBe("2");
  });

  it("enriches runs.length from loaded manifest when experiment_id matches", () => {
    const primary = cohort.manifest_refs[0]!.manifest_ref;
    const secondary = cohort.manifest_refs[1]!.manifest_ref;
    const loaded: ExperimentManifest = {
      schema: "rt_experiment_manifest_v1",
      experiment_id: cohort.manifest_refs[0]!.experiment_id,
      created_at_utc: "2026-01-01T00:00:00Z",
      governance_banner: "RT EXPERIMENT — explanatory compare only; not operational authority",
      runs: [
        {
          run_id: "r1",
          label: "r1",
          session_id: "s1",
          recorded_at_utc: "2026-01-01T00:00:00Z",
          spec_fingerprint: "fp-a",
        },
        {
          run_id: "r2",
          label: "r2",
          session_id: "s2",
          recorded_at_utc: "2026-01-01T00:00:01Z",
          spec_fingerprint: "fp-b",
          capture_candidate_id: "cap-1",
        },
      ],
    };
    const rows = buildMultiManifestDiff({
      cohort,
      primaryManifestRef: primary,
      secondaryManifestRef: secondary,
      loadedManifest: loaded,
    });
    const runsRow = rows.find((r) => r.field === "runs.length");
    expect(runsRow?.primary).toBe("2");
    expect(runsRow?.note).toContain("cohort hint 4");
    const captureRow = rows.find((r) => r.field === "capture_count");
    expect(captureRow?.primary).toBe("1");
    const fpRow = rows.find((r) => r.field === "spec_fingerprint");
    expect(fpRow?.primary).toContain("fp-a");
    expect(fpRow?.source).toBe("loaded_manifest");
  });

  it("returns empty when refs missing", () => {
    expect(
      buildMultiManifestDiff({
        cohort,
        primaryManifestRef: null,
        secondaryManifestRef: cohort.manifest_refs[1]!.manifest_ref,
      }),
    ).toEqual([]);
  });

  it("builds summary chips for primary and secondary", () => {
    const chips = buildManifestSummaryChips({
      cohort,
      primaryRef: cohort.manifest_refs[0]!.manifest_ref,
      secondaryRef: cohort.manifest_refs[1]!.manifest_ref,
    });
    expect(chips).toHaveLength(2);
    expect(chips[0]?.role).toBe("primary");
    expect(chips[1]?.role).toBe("secondary");
  });
});
