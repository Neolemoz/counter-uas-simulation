import { describe, expect, it } from "vitest";
import {
  DEFAULT_PLANNING_EXTENT_ID,
  LEGACY_PLANNING_EXTENTS,
  PLANNING_EXTENT_GOVERNANCE_COPY,
  UNIFIED_PLANNING_WORLD,
  planningExtentById,
  planningExtentMetadata,
  unifiedPlanningWorld,
} from "./planningWorld";
import { WORLD_BOUNDS } from "@/world/bounds";

describe("planningWorld", () => {
  it("defines unified 7 km world as the default Planning authority surface", () => {
    expect(unifiedPlanningWorld()).toEqual(UNIFIED_PLANNING_WORLD);
    expect(planningExtentById(DEFAULT_PLANNING_EXTENT_ID).planning_extent_label).toBe(
      "Unified 7 km World",
    );
    expect(UNIFIED_PLANNING_WORLD.planning_extent_radius_m).toBe(7000);
  });

  it("retains legacy extent metadata for imported snapshots only", () => {
    expect(LEGACY_PLANNING_EXTENTS.map((extent) => extent.planning_extent_radius_m)).toEqual([
      5_000,
      10_000,
      20_000,
    ]);
    const metadata = planningExtentMetadata(planningExtentById("planning_20km"));
    expect(metadata).toEqual({
      planning_extent_id: "planning_20km",
      planning_extent_radius_m: 20_000,
      planning_extent_label: "20 km Planning World (legacy)",
    });
  });

  it("returns stable extent metadata without mutating runtime bounds", () => {
    const before = JSON.stringify(WORLD_BOUNDS);
    planningExtentMetadata(planningExtentById("planning_20km"));
    expect(JSON.stringify(WORLD_BOUNDS)).toBe(before);
    expect(WORLD_BOUNDS.x.max).toBe(7000);
  });

  it("states unified world governance boundaries", () => {
    expect(PLANNING_EXTENT_GOVERNANCE_COPY).toContain("unified 7 km world");
    expect(PLANNING_EXTENT_GOVERNANCE_COPY).toContain("Legacy planning extent IDs");
  });
});
