import { describe, expect, it } from "vitest";
import type { MirrorEntity } from "./entityMarkers";
import { DEFAULT_TERRAIN_LAYERS } from "./terrainLayers";
import { defaultVisibilityFromRegistry } from "./visualLayerRegistry";
import { shouldUseLegacyLosPath, syncStackedLosPresentation } from "./stackedLosPresentation";

const entity: MirrorEntity = {
  entity_id: "e1",
  entity_type: "drone",
  pose: { x: 0, y: 0, z: 10 },
};

describe("stackedLosPresentation", () => {
  it("uses legacy LOS path when stacked off and terrain on", () => {
    const v = defaultVisibilityFromRegistry();
    expect(shouldUseLegacyLosPath(v, DEFAULT_TERRAIN_LAYERS, entity)).toBe(true);
  });

  it("skips legacy LOS when stacked on", () => {
    const v = { ...defaultVisibilityFromRegistry(), showStackedLos: true };
    expect(shouldUseLegacyLosPath(v, DEFAULT_TERRAIN_LAYERS, entity)).toBe(false);
  });

  it("noops on null viewer", () => {
    const v = defaultVisibilityFromRegistry();
    expect(() =>
      syncStackedLosPresentation(null, entity, [entity], v, DEFAULT_TERRAIN_LAYERS),
    ).not.toThrow();
  });
});
