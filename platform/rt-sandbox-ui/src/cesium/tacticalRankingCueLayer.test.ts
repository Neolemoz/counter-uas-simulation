import { describe, expect, it } from "vitest";
import type {
  TacticalRecommendationPayload,
  TacticalStatePayload,
} from "@/bridge/tacticalCommands";
import {
  formatTacticalRankingSummary,
  resolveTacticalRankingCues,
} from "./tacticalRankingCueLayer";

describe("tacticalRankingCueLayer", () => {
  it("renders ranked list cues when ranked_candidates exists", () => {
    const state = {
      ranked_candidates: [
        { entity_id: "tgt-a", rank: 1 },
        { entity_id: "tgt-b", rank: 2 },
      ],
    } as TacticalStatePayload & {
      ranked_candidates: { entity_id: string; rank: number }[];
    };
    const resolution = resolveTacticalRankingCues(state, null);
    expect(resolution.mode).toBe("ranked_list");
    expect(resolution.cues).toEqual([
      { entityId: "tgt-a", rank: 1, label: "#1" },
      { entityId: "tgt-b", rank: 2, label: "#2" },
    ]);
    expect(formatTacticalRankingSummary(resolution)).toMatch(/Ranked cues/);
  });

  it("renders a single recommendation cue when no ranked list exists", () => {
    const recommendation: TacticalRecommendationPayload = {
      recommended_target_id: "tgt-rec",
      recommended_interceptor_id: "int-rec",
      tti_s: 14.2,
    };
    const resolution = resolveTacticalRankingCues(null, recommendation);
    expect(resolution.mode).toBe("recommendation_only");
    expect(resolution.cues).toEqual([
      {
        entityId: "tgt-rec",
        rank: 1,
        label: "rec #1 · 14s · cue only",
      },
    ]);
    expect(formatTacticalRankingSummary(resolution)).toMatch(/not full ranking/);
  });

  it("falls back to assigned target for recommendation cue", () => {
    const state: TacticalStatePayload = {
      assigned_target_id: "tgt-assigned",
      tti_s: 9,
    };
    const resolution = resolveTacticalRankingCues(state, null);
    expect(resolution.mode).toBe("recommendation_only");
    expect(resolution.cues[0]?.entityId).toBe("tgt-assigned");
    expect(resolution.cues[0]?.label).toMatch(/^rec #1/);
  });
});
