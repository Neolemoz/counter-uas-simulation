import { describe, expect, it } from "vitest";
import { COMPARE_MODE_COACH, compareModeCoachLine } from "./compareModeCoach";

describe("compareModeCoach", () => {
  it("defines coach for each compare mode", () => {
    expect(COMPARE_MODE_COACH.pairwise_pinned).toContain("Two runs");
    expect(COMPARE_MODE_COACH.multi_manifest_diff).toContain("metadata");
    expect(compareModeCoachLine("cohort_matrix")).toContain("parameter_matrix");
  });
});
