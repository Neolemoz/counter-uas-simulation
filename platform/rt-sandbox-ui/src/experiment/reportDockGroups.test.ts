import { describe, expect, it } from "vitest";
import {
  defaultGroupExpanded,
  defaultGroupExpandedMap,
  REPORT_DOCK_GROUPS,
} from "./reportDockGroups";

describe("reportDockGroups", () => {
  it("defines four dock groups", () => {
    expect(REPORT_DOCK_GROUPS).toHaveLength(4);
    expect(REPORT_DOCK_GROUPS.map((g) => g.label)).toEqual([
      "Analytics",
      "Continuity",
      "Metrics",
      "Fidelity",
    ]);
  });

  it("expands analytics when on f1 step", () => {
    expect(defaultGroupExpanded("analytics", "f1_analytics")).toBe(true);
    expect(defaultGroupExpanded("analytics", "compare")).toBe(false);
  });

  it("collapses continuity until step 3", () => {
    expect(defaultGroupExpanded("continuity", "f3_continuity")).toBe(true);
    expect(defaultGroupExpanded("continuity", "f1_analytics")).toBe(false);
  });

  it("builds expanded map", () => {
    const map = defaultGroupExpandedMap("f5_metrics");
    expect(map.metrics).toBe(true);
    expect(map.analytics).toBe(false);
  });
});
