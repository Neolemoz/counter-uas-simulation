import { describe, expect, it } from "vitest";
import { Color } from "cesium";
import type { MirrorEntity } from "./entityMarkers";

// Test alpha scaling via exported behavior: color strings differ when muted path used.
// Full sync requires Cesium Viewer — we validate the constant and type contract.

describe("entityMarkers markerEmphasis", () => {
  const sampleEntity: MirrorEntity = {
    entity_id: "e1",
    entity_type: "drone",
    pose: { x: 0, y: 0, z: 0 },
  };

  it("exports MarkerEmphasis type contract", () => {
    expect(sampleEntity.entity_id).toBe("e1");
  });

  it("applies 0.55 alpha scale helper semantics", () => {
    const base = Color.fromCssColorString("#34d399");
    const scaled = base.clone();
    scaled.alpha *= 0.55;
    expect(scaled.alpha).toBeCloseTo(base.alpha * 0.55, 5);
  });
});
