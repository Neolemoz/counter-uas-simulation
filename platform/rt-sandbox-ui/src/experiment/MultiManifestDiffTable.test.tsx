import { readFileSync } from "node:fs";
import { join } from "node:path";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { experimentCohortIndexSchema } from "./cohortSchema";
import { MultiManifestDiffTable } from "./MultiManifestDiffTable";
import { MULTI_MANIFEST_DIFF_BANNER } from "./multiManifestDiff";

const REPO_ROOT = join(import.meta.dirname, "../../../..");
const cohort = experimentCohortIndexSchema.parse(
  JSON.parse(
    readFileSync(
      join(REPO_ROOT, "fixtures/rt_experiments/x2_cohort_index_example.json"),
      "utf-8",
    ),
  ),
);

describe("MultiManifestDiffTable", () => {
  it("shows guidance when refs missing", () => {
    const markup = renderToStaticMarkup(
      <MultiManifestDiffTable
        cohort={cohort}
        primaryManifestRef={null}
        secondaryManifestRef={null}
      />,
    );
    expect(markup).toContain(MULTI_MANIFEST_DIFF_BANNER);
    expect(markup).toContain("manifest roster");
  });

  it("renders diff table for cohort manifests", () => {
    const markup = renderToStaticMarkup(
      <MultiManifestDiffTable
        cohort={cohort}
        primaryManifestRef={cohort.manifest_refs[0]!.manifest_ref}
        secondaryManifestRef={cohort.manifest_refs[1]!.manifest_ref}
      />,
    );
    expect(markup).toContain("experiment_id");
    expect(markup).toContain("manifest-summary-chips");
    expect(markup).toContain("compare-status-chip");
    expect(markup).toContain("data-status=\"divergent\"");
    expect(markup).toContain("Primary");
    expect(markup).toContain("Secondary");
  });
});
