import { describe, expect, it } from "vitest";
import {
  compareBadges,
  entityCount,
  sideFromLive,
  sideFromPinnedRun,
} from "./experimentCompare";
import type { ExperimentRun } from "./experimentSchema";

describe("experimentCompare", () => {
  const baseRun: ExperimentRun = {
    run_id: "a",
    label: "A",
    session_id: "sess-a",
    recorded_at_utc: "2026-05-26T00:00:00Z",
    snapshot: {
      tactical_state: { tactical_mode: "manual", tti_s: 10 },
      world_summary: { entity_count: 3 },
    },
  };

  it("builds pinned side with capture source when capture id present", () => {
    const run: ExperimentRun = {
      ...baseRun,
      capture_candidate_id: "cap-1",
    };
    const side = sideFromPinnedRun(run);
    expect(side.source).toBe("capture");
    expect(side.tactical?.tactical_mode).toBe("manual");
  });

  it("emits mode_changed and tti_delta badges", () => {
    const a = sideFromPinnedRun(baseRun);
    const b = sideFromPinnedRun({
      ...baseRun,
      run_id: "b",
      snapshot: {
        tactical_state: { tactical_mode: "assisted", tti_s: 20 },
        world_summary: { entity_count: 1 },
      },
    });
    const badges = compareBadges(a, b);
    expect(badges.map((x) => x.id)).toContain("mode_changed");
    expect(badges.map((x) => x.id)).toContain("tti_delta");
  });

  it("counts entities from world_summary", () => {
    expect(entityCount({ entity_count: 5 })).toBe(5);
    expect(entityCount(null)).toBe(0);
  });

  it("live side uses live source", () => {
    const side = sideFromLive("slot-1", "s1", {
      tactical_state: { tactical_mode: "manual" },
    });
    expect(side.source).toBe("live");
  });
});
