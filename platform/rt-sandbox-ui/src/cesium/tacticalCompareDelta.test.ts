import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import { deriveTacticalCompareDeltaLabels } from "./tacticalCompareDelta";

describe("tacticalCompareDelta", () => {
  it("formats ΔTTI when tti changes", () => {
    const current: TacticalStatePayload = {
      tti_s: 5.3,
      selected_target_id: "tgt-a",
    };
    const compare: TacticalStatePayload = {
      tti_s: 4.1,
      selected_target_id: "tgt-a",
    };
    const labels = deriveTacticalCompareDeltaLabels(current, compare);
    expect(labels.timingLine).toBe("ΔTTI +1.2s");
    expect(labels.block).toBe("ΔTTI +1.2s");
  });

  it("notes target switched", () => {
    const labels = deriveTacticalCompareDeltaLabels(
      { selected_target_id: "tgt-b", tti_s: 3 },
      { selected_target_id: "tgt-a", tti_s: 3 },
    );
    expect(labels.targetLine).toBe("target switched");
    expect(labels.block).toContain("target switched");
  });

  it("returns null block when no delta", () => {
    const state: TacticalStatePayload = {
      selected_target_id: "tgt-a",
      tti_s: 2,
    };
    expect(deriveTacticalCompareDeltaLabels(state, state).block).toBeNull();
  });
});
