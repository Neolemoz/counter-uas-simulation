import { describe, expect, it } from "vitest";
import { filterLosSegmentsForT } from "./losSegmentLayer";
import type { ReplaySaBundle } from "@/replay/bundleSchema";

const baseBundle = {
  los_segments: [
    {
      segment_id: "a",
      from_entity_id: "radar",
      to_track_id: "threat",
      status: "visible" as const,
      polyline_enu_m: [
        [0, 0, 0],
        [1, 1, 1],
      ],
      caveat: "test",
      t: 5,
    },
    {
      segment_id: "b",
      from_entity_id: "eoir",
      to_track_id: "threat",
      status: "terrain_blocked" as const,
      polyline_enu_m: [
        [0, 0, 0],
        [2, 2, 2],
      ],
      caveat: "test",
      t: 7,
    },
  ],
} as Pick<ReplaySaBundle, "los_segments">;

describe("filterLosSegmentsForT", () => {
  it("returns segments matching currentT when present", () => {
    const out = filterLosSegmentsForT(baseBundle as ReplaySaBundle, 5);
    expect(out).toHaveLength(1);
    expect(out[0].segment_id).toBe("a");
  });

  it("falls back to all segments when no match at t", () => {
    const out = filterLosSegmentsForT(baseBundle as ReplaySaBundle, 99);
    expect(out.length).toBeGreaterThan(0);
  });
});
