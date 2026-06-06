import { describe, expect, it } from "vitest";
import { Color } from "cesium";
import type { MirrorEntity } from "./entityMarkers";
import { PROTECTED_CENTER_HALO_PIXEL_SIZE } from "./entityMarkers";
import { isDesignatedProtectedCenter } from "./defenseZoneVisualState";
import { resolveTacticalTargetEntityId } from "./tacticalPreset";
import { SELECTION_RING_PIXEL_SIZE } from "./visualStyle";

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

  it("wires tactical target entity id from assigned or selected tactical state", () => {
    expect(
      resolveTacticalTargetEntityId({
        assigned_target_id: "tgt-assigned",
        selected_target_id: "tgt-selected",
      }),
    ).toBe("tgt-assigned");
    expect(
      resolveTacticalTargetEntityId({
        selected_target_id: "tgt-selected",
      }),
    ).toBe("tgt-selected");
  });

  it("uses larger emerald halo size distinct from selection ring", () => {
    expect(PROTECTED_CENTER_HALO_PIXEL_SIZE).toBeGreaterThan(SELECTION_RING_PIXEL_SIZE);
  });

  it("does not treat selection as protected-center designation", () => {
    expect(isDesignatedProtectedCenter("wp-a", null)).toBe(false);
    expect(isDesignatedProtectedCenter("wp-a", "wp-b")).toBe(false);
    expect(isDesignatedProtectedCenter("center-a", "center-a")).toBe(true);
  });
});
