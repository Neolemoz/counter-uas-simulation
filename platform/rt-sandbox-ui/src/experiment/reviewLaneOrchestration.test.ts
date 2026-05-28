import { describe, expect, it } from "vitest";
import {
  applyStepWithCompareMode,
  compareModeActivations,
  stepPanelTargets,
} from "./reviewLaneOrchestration";

describe("reviewLaneOrchestration", () => {
  it("maps f1 step to analytics", () => {
    expect(stepPanelTargets("f1_analytics")).toEqual({
      analytics: true,
      continuity: false,
      compare: false,
      f5: false,
    });
  });

  it("maps pairwise compare mode", () => {
    expect(compareModeActivations("pairwise_pinned")).toEqual({
      compareModeActive: true,
      f5Active: false,
    });
  });

  it("maps extended compare to f5", () => {
    expect(compareModeActivations("extended_n_run")).toEqual({
      compareModeActive: false,
      f5Active: true,
    });
  });

  it("merges compare step with mode", () => {
    const merged = applyStepWithCompareMode("compare", "cohort_matrix");
    expect(merged.f5Active).toBe(true);
    expect(merged.compareModeActive).toBe(false);
  });
});
