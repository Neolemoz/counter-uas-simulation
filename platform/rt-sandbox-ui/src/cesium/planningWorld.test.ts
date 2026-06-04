import { describe, expect, it } from "vitest";
import {
  DEFAULT_PLANNING_EXTENT_ID,
  PLANNING_EXTENT_GOVERNANCE_COPY,
  PLANNING_EXTENTS,
  planningExtentById,
  planningExtentMetadata,
} from "./planningWorld";
import { WORLD_BOUNDS } from "@/world/bounds";

describe("planningWorld", () => {
  it("defines Planning-only large-area extent presets", () => {
    expect(PLANNING_EXTENTS.map((extent) => extent.planning_extent_radius_m)).toEqual([
      5_000,
      10_000,
      20_000,
    ]);
    expect(planningExtentById(DEFAULT_PLANNING_EXTENT_ID).planning_extent_label).toBe(
      "10 km Planning World",
    );
  });

  it("returns stable extent metadata without mutating runtime bounds", () => {
    const before = JSON.stringify(WORLD_BOUNDS);
    const metadata = planningExtentMetadata(planningExtentById("planning_20km"));

    expect(metadata).toEqual({
      planning_extent_id: "planning_20km",
      planning_extent_radius_m: 20_000,
      planning_extent_label: "20 km Planning World",
    });
    expect(JSON.stringify(WORLD_BOUNDS)).toBe(before);
    expect(WORLD_BOUNDS.x.max).toBe(500);
  });

  it("states Planning extent governance boundaries", () => {
    expect(PLANNING_EXTENT_GOVERNANCE_COPY).toContain("not runtime authority");
    expect(PLANNING_EXTENT_GOVERNANCE_COPY).toContain("bridge bounds");
    expect(PLANNING_EXTENT_GOVERNANCE_COPY).toContain("MC execution authority");
  });
});

