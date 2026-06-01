import { describe, expect, it } from "vitest";
import { DEFENSE_ZONE_LEVELS } from "./defenseZoneLayer";
import {
  zoneBoundaryPositionsGrounded,
  labelAzimuthDegForDefenseZone,
  metersToSvg,
} from "./defenseZoneGeometry";

describe("defenseZoneLayer", () => {
  it("orders defense zone edge strength core > mid > warning", () => {
    const alphas = DEFENSE_ZONE_LEVELS.map((l) => l.alpha);
    expect(alphas[0]).toBeGreaterThan(alphas[1]);
    expect(alphas[1]).toBeGreaterThan(alphas[2]);
  });

  it("uses red / yellow / blue hierarchy for core / engagement / warning", () => {
    expect(DEFENSE_ZONE_LEVELS[0].label).toBe("Core");
    expect(DEFENSE_ZONE_LEVELS[0].fillRgb).toContain("248");
    expect(DEFENSE_ZONE_LEVELS[1].label).toBe("Engagement");
    expect(DEFENSE_ZONE_LEVELS[1].fillRgb).toContain("250");
    expect(DEFENSE_ZONE_LEVELS[2].label).toBe("Warning");
    expect(DEFENSE_ZONE_LEVELS[2].fillRgb).toContain("96");
  });

  it("staggers defense labels by bearing", () => {
    const bearings = (["core", "mid", "warning"] as const).map(
      labelAzimuthDegForDefenseZone,
    );
    expect(new Set(bearings).size).toBe(3);
  });
});

describe("defenseZoneGeometry", () => {
  it("builds grounded circle and rectangle boundaries", () => {
    const circle = zoneBoundaryPositionsGrounded("circle", 0, 0, 100, 0.4, 32);
    const rect = zoneBoundaryPositionsGrounded("rectangle", 0, 0, 100, 0.4);
    expect(circle.length).toBeGreaterThan(rect.length);
    expect(rect[0]).toBeDefined();
    expect(rect[4]).toEqual(rect[0]);
  });

  it("scales meters to svg pixels", () => {
    expect(metersToSvg(100, 14)).toBeGreaterThan(0);
  });
});
