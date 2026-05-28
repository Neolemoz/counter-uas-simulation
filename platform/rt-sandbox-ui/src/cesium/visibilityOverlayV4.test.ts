import { describe, expect, it } from "vitest";
import { DEFAULT_TERRAIN_LAYERS } from "./terrainLayers";
import { defaultVisibilityFromRegistry } from "./visualLayerRegistry";
import {
  deriveVisibilityOverlayV4Hints,
  visibilityOverlayV4SummaryLine,
} from "./visibilityOverlayV4";
import type { MirrorEntity } from "./entityMarkers";

const selected: MirrorEntity = {
  entity_id: "radar-primary",
  entity_type: "radar",
  pose: { x: 0, y: 0, z: 25, yaw_deg: 20 },
};

const peer: MirrorEntity = {
  entity_id: "drone-peer",
  entity_type: "drone",
  pose: { x: 120, y: 40, z: 80, yaw_deg: 0 },
};

describe("visibilityOverlayV4", () => {
  it("keeps P1 overlays default-off", () => {
    const hints = deriveVisibilityOverlayV4Hints({
      visibility: defaultVisibilityFromRegistry(),
      selected,
      entities: [selected, peer],
      terrainLayers: DEFAULT_TERRAIN_LAYERS,
    });

    expect(hints).toEqual([]);
    expect(visibilityOverlayV4SummaryLine(hints)).toMatch(/off/);
  });

  it("derives explanatory visibility and terrain hints without authority semantics", () => {
    const visibility = {
      ...defaultVisibilityFromRegistry(),
      showVisibilityCorridorV4: true,
      showOcclusionBandsV4: true,
      showTerrainRelationLabelsV4: true,
    };

    const hints = deriveVisibilityOverlayV4Hints({
      visibility,
      selected,
      entities: [selected, peer],
      terrainLayers: DEFAULT_TERRAIN_LAYERS,
    });

    expect(hints.map((h) => h.kind)).toContain("visibility_corridor");
    expect(hints.map((h) => h.kind)).toContain("occlusion_band");
    expect(hints.map((h) => h.kind)).toContain("terrain_relation");
    expect(hints.every((h) => h.explanatory)).toBe(true);
    expect(visibilityOverlayV4SummaryLine(hints)).toMatch(/heuristic and explanatory only/);
  });

  it("surfaces compare emphasis as visual-only cognition", () => {
    const visibility = {
      ...defaultVisibilityFromRegistry(),
      showCompareEmphasisV4: true,
    };

    const hints = deriveVisibilityOverlayV4Hints({
      visibility,
      selected: null,
      entities: [],
      terrainLayers: DEFAULT_TERRAIN_LAYERS,
    });

    expect(hints).toHaveLength(1);
    expect(hints[0].kind).toBe("compare_emphasis");
    expect(hints[0].label).toMatch(/visually only/);
  });
});
