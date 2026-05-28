import { describe, expect, it } from "vitest";
import { cellCompareStatus, pairwiseCompareStatus } from "./compareBadgeStatus";

describe("compareBadgeStatus", () => {
  it("pairwise aligned when no badges", () => {
    expect(pairwiseCompareStatus([])).toBe("aligned");
  });

  it("pairwise divergent when badges present", () => {
    expect(pairwiseCompareStatus([{ id: "x", label: "mode_changed" }])).toBe("divergent");
  });

  it("cell missing without value", () => {
    expect(cellCompareStatus(false)).toBe("missing");
  });
});
