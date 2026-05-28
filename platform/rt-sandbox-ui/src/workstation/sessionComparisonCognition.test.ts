import { describe, expect, it } from "vitest";
import {
  deriveSessionComparisonVisualRows,
  sessionComparisonSummaryLine,
} from "./sessionComparisonCognition";

describe("sessionComparisonCognition", () => {
  it("keeps only the selected session commandable", () => {
    const rows = deriveSessionComparisonVisualRows({
      activeSessionId: "session-a",
      orderedSessionIds: ["session-a", "session-b", "session-c"],
      comparisonGhostsEnabled: true,
      sessionContrastEnabled: true,
    });

    expect(rows.map((r) => r.role)).toEqual(["selected", "comparison", "comparison"]);
    expect(rows.filter((r) => r.commandable).map((r) => r.sessionId)).toEqual(["session-a"]);
    expect(rows.slice(1).every((r) => r.displayMode === "muted")).toBe(true);
  });

  it("uses tab-only background rows when comparison ghosts are off", () => {
    const rows = deriveSessionComparisonVisualRows({
      activeSessionId: "session-a",
      orderedSessionIds: ["session-a", "session-b"],
      comparisonGhostsEnabled: false,
      sessionContrastEnabled: true,
    });

    expect(rows[1].role).toBe("background");
    expect(rows[1].displayMode).toBe("tab-only");
    expect(sessionComparisonSummaryLine(rows)).toMatch(/tab-only/);
  });

  it("dims secondary rows when compare emphasis is enabled", () => {
    const rows = deriveSessionComparisonVisualRows({
      activeSessionId: "session-a",
      orderedSessionIds: ["session-a", "session-b"],
      comparisonGhostsEnabled: true,
      sessionContrastEnabled: true,
      compareEmphasisEnabled: true,
    });

    expect(rows[1].role).toBe("comparison");
    expect(rows[1].displayMode).toBe("dimmed");
    expect(rows[1].commandable).toBe(false);
  });
});
