import { describe, expect, it } from "vitest";
import {
  clusterRadiusMForCamera,
  computeMarkerLabelLayouts,
  type MarkerLabelLayoutInput,
} from "./markerLabelLayout";

function marker(
  id: string,
  x: number,
  y: number,
  overrides: Partial<MarkerLabelLayoutInput> = {},
): MarkerLabelLayoutInput {
  return {
    entityId: id,
    entityType: "drone",
    x,
    y,
    selected: false,
    hovered: false,
    ...overrides,
  };
}

describe("markerLabelLayout", () => {
  it("widens cluster radius when camera is farther away", () => {
    expect(clusterRadiusMForCamera(5000)).toBeGreaterThan(clusterRadiusMForCamera(800));
  });

  it("fans offsets for markers within cluster radius", () => {
    const layouts = computeMarkerLabelLayouts(
      [marker("a", 0, 0), marker("b", 5, 0), marker("c", -4, 3)],
      1500,
    );
    const a = layouts.get("a");
    const b = layouts.get("b");
    expect(a).toBeDefined();
    expect(b).toBeDefined();
    expect(a!.offsetX !== b!.offsetX || a!.offsetY !== b!.offsetY).toBe(true);
  });

  it("keeps selected marker on a centered lift offset in a cluster", () => {
    const layouts = computeMarkerLabelLayouts(
      [
        marker("a", 0, 0, { selected: true }),
        marker("b", 4, 0),
      ],
      1200,
    );
    expect(layouts.get("a")).toEqual({ offsetX: 0, offsetY: -22, glyphOnly: false });
  });

  it("uses glyph-only labels for dense distant clusters", () => {
    const layouts = computeMarkerLabelLayouts(
      [
        marker("a", 0, 0),
        marker("b", 8, 0),
        marker("c", -6, 4),
      ],
      3000,
    );
    expect(layouts.get("a")?.glyphOnly).toBe(true);
    expect(layouts.get("b")?.glyphOnly).toBe(true);
  });
});
