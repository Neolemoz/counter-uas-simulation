import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import { parseTacticalTargetRanking, rankLabelFor } from "./tacticalTargetRanking";

describe("tacticalTargetRanking", () => {
  it("parses ranked target list from telemetry", () => {
    const state = {
      target_ranking: [
        { entity_id: "tgt-a", rank: 1 },
        { entity_id: "tgt-b", rank: 2 },
        { entity_id: "tgt-c", rank: 3 },
      ],
    } as TacticalStatePayload & {
      target_ranking: { entity_id: string; rank: number }[];
    };
    expect(parseTacticalTargetRanking(state, "tgt-a")).toEqual([
      { entityId: "tgt-a", rank: 1 },
      { entityId: "tgt-b", rank: 2 },
      { entityId: "tgt-c", rank: 3 },
    ]);
  });

  it("falls back to selected target as #1", () => {
    expect(parseTacticalTargetRanking(null, "tgt-1")).toEqual([
      { entityId: "tgt-1", rank: 1 },
    ]);
  });

  it("caps at three rank labels", () => {
    const state = {
      ranked_targets: ["a", "b", "c", "d", "e"],
    } as TacticalStatePayload & { ranked_targets: string[] };
    expect(parseTacticalTargetRanking(state, null)).toHaveLength(3);
  });

  it("formats rank labels", () => {
    expect(rankLabelFor(1)).toBe("#1");
    expect(rankLabelFor(3)).toBe("#3");
  });
});
