import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import {
  deriveTacticalCompareDeltaLabels,
  formatTacticalCompareSummary,
} from "./tacticalCompareDelta";

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

  it("notes target mismatch", () => {
    const labels = deriveTacticalCompareDeltaLabels(
      { selected_target_id: "tgt-b", tti_s: 3 },
      { selected_target_id: "tgt-a", tti_s: 3 },
    );
    expect(labels.targetLine).toBe("target mismatch");
    expect(labels.block).toContain("target mismatch");
  });

  it("notes assignment mismatch", () => {
    const labels = deriveTacticalCompareDeltaLabels(
      {
        assigned_interceptor_id: "int-b",
        assigned_target_id: "tgt-a",
        tti_s: 3,
      },
      {
        assigned_interceptor_id: "int-a",
        assigned_target_id: "tgt-a",
        tti_s: 3,
      },
    );
    expect(labels.assignmentLine).toBe("assignment mismatch");
    expect(labels.block).toContain("assignment mismatch");
  });

  it("returns null block when no delta", () => {
    const state: TacticalStatePayload = {
      selected_target_id: "tgt-a",
      tti_s: 2,
    };
    expect(deriveTacticalCompareDeltaLabels(state, state).block).toBeNull();
  });

  it("formats panel summary with delta lines", () => {
    const summary = formatTacticalCompareSummary({
      source: "session",
      compareSessionId: "session-background-001",
      deltas: deriveTacticalCompareDeltaLabels(
        { tti_s: 5, selected_target_id: "tgt-b" },
        { tti_s: 3, selected_target_id: "tgt-a" },
      ),
      hasCompareGeometry: true,
    });
    expect(summary).toMatch(/background/);
    expect(summary).toMatch(/ΔTTI/);
    expect(summary).toMatch(/display only/);
  });
});
