import { describe, expect, it } from "vitest";
import {
  MULTI_MANIFEST_FIELD_ORDER,
  sortMultiManifestRows,
  summarizeMultiManifestDiffStatus,
  truncateManifestRef,
} from "./multiManifestDiffColumns";
import type { MultiManifestDiffRow } from "./multiManifestDiff";

describe("multiManifestDiffColumns", () => {
  it("defines normative field order", () => {
    expect(MULTI_MANIFEST_FIELD_ORDER[0]).toBe("experiment_id");
    expect(MULTI_MANIFEST_FIELD_ORDER).toHaveLength(6);
  });

  it("sortMultiManifestRows orders by field catalog", () => {
    const rows: MultiManifestDiffRow[] = [
      {
        field: "tag_overlap",
        primary: "a",
        secondary: "—",
        source: "overlap",
        compare_status: "explanatory",
      },
      {
        field: "experiment_id",
        primary: "e1",
        secondary: "e2",
        source: "cohort_index",
        compare_status: "divergent",
      },
    ];
    const sorted = sortMultiManifestRows(rows);
    expect(sorted[0]!.field).toBe("experiment_id");
    expect(sorted[1]!.field).toBe("tag_overlap");
  });

  it("summarizeMultiManifestDiffStatus counts statuses", () => {
    const rows: MultiManifestDiffRow[] = [
      {
        field: "a",
        primary: "1",
        secondary: "1",
        source: "cohort_index",
        compare_status: "aligned",
      },
      {
        field: "b",
        primary: "1",
        secondary: "2",
        source: "cohort_index",
        compare_status: "divergent",
      },
    ];
    expect(summarizeMultiManifestDiffStatus(rows)).toContain("1 aligned");
    expect(summarizeMultiManifestDiffStatus(rows)).toContain("1 divergent");
  });

  it("truncateManifestRef shortens long refs", () => {
    const long = "runs/rt_sandbox/experiments/foo/manifest.json";
    expect(truncateManifestRef(long, 20).endsWith("…")).toBe(true);
  });
});
