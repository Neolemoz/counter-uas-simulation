import { describe, expect, it } from "vitest";
import {
  describeCommandIntent,
  describeMirrorLag,
  editSafetySummary,
  formatCommandResult,
} from "./cognition";

describe("editing cognition", () => {
  it("describes command intent", () => {
    expect(describeCommandIntent("spawn_entity")).toContain("command authoritative");
  });

  it("describes mirror lag when pending", () => {
    expect(describeMirrorLag(true)).toContain("lag");
  });

  it("summarizes safety caps", () => {
    const s = editSafetySummary({ entity_count: 2, by_type: { drone: 1 } });
    expect(s.entityCount).toBe("2/32");
    expect(s.byType.drone).toBe("1/8");
  });

  it("formats command result", () => {
    expect(formatCommandResult(false, "INVALID_POSE")).toContain("INVALID_POSE");
  });
});
