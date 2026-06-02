import { describe, expect, it } from "vitest";
import { targetIdFromTacticalState } from "./tacticalSelectionEmphasisLayer";

describe("tacticalSelectionEmphasisLayer", () => {
  it("prefers assigned target over selected target for visual emphasis", () => {
    expect(
      targetIdFromTacticalState({
        assigned_target_id: "assigned-target",
        selected_target_id: "selected-target",
      }),
    ).toBe("assigned-target");
  });

  it("falls back to selected target when no assignment exists", () => {
    expect(targetIdFromTacticalState({ selected_target_id: "selected-target" })).toBe(
      "selected-target",
    );
  });
});
