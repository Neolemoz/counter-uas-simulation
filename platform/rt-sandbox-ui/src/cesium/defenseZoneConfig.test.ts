import { describe, expect, it } from "vitest";
import {
  DEFAULT_DEFENSE_ZONE_CONFIG,
  isProtectedAsset,
  normalizedDefenseZoneConfig,
  normalizedDefenseZoneSizes,
} from "./defenseZoneConfig";

describe("defenseZoneConfig", () => {
  it("identifies protected waypoint assets", () => {
    expect(isProtectedAsset("waypoint_marker")).toBe(true);
    expect(isProtectedAsset("radar")).toBe(false);
  });

  it("normalizes defense zone size ordering", () => {
    const sizes = normalizedDefenseZoneSizes({
      coreM: 80,
      engageM: 70,
      warningM: 60,
    });
    expect(sizes.engageM).toBeGreaterThan(sizes.coreM);
    expect(sizes.warningM).toBeGreaterThan(sizes.engageM);
  });

  it("defaults shape to circle and keeps labels on", () => {
    const config = normalizedDefenseZoneConfig({});
    expect(config.shape).toBe("circle");
    expect(config.showLabels).toBe(true);
    expect(config.sizes).toEqual(DEFAULT_DEFENSE_ZONE_CONFIG.sizes);
  });

  it("preserves rectangle shape when requested", () => {
    const config = normalizedDefenseZoneConfig({ shape: "rectangle" });
    expect(config.shape).toBe("rectangle");
  });
});
