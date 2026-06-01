import { describe, expect, it } from "vitest";
import {
  markerCellOffset,
  markerVisualStyle,
  MARKER_HIT_RADIUS_FACTOR,
  resolveEntityType,
  spawnTypeLabel,
} from "./worldEditorVisuals";

describe("worldEditorVisuals", () => {
  it("uses larger hit radius than cell half-width", () => {
    expect(MARKER_HIT_RADIUS_FACTOR).toBeGreaterThan(1);
  });

  it("emphasizes selected markers over hover", () => {
    const selected = markerVisualStyle("drone", true, false, false);
    const hovered = markerVisualStyle("drone", false, true, false);
    expect(selected.strokeWidth).toBeGreaterThan(hovered.strokeWidth);
    expect(selected.outerRingRadiusFactor).toBeGreaterThan(0);
  });

  it("resolves unknown types to drone palette", () => {
    expect(resolveEntityType("unknown")).toBe("drone");
  });

  it("labels spawn preview from entity catalog", () => {
    expect(spawnTypeLabel("radar")).toBe("Radar");
  });

  it("fans stacked markers within a cell", () => {
    const a = markerCellOffset(0, 3, 30);
    const b = markerCellOffset(1, 3, 30);
    expect(a.dx !== b.dx || a.dy !== b.dy).toBe(true);
  });
});
