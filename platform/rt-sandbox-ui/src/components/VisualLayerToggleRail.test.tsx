import { describe, expect, it } from "vitest";
import { groupLayersForUi, CANONICAL_VISUAL_LAYER_REGISTRY } from "@/cesium/visualLayerRegistry";
import { partitionLayersForCompactUi } from "./VisualLayerToggleRail";

describe("VisualLayerToggleRail", () => {
  it("surfaces terrain, sensor domes, and labels as primary toggles", () => {
    const { primary, advanced } = partitionLayersForCompactUi(
      groupLayersForUi(CANONICAL_VISUAL_LAYER_REGISTRY),
    );
    expect(primary.map((layer) => layer.layer_id)).toEqual([
      "terrain_mesh",
      "sensor_domes",
      "entity_labels",
    ]);
    expect(advanced.some((group) => group.groupId === "visibility_context")).toBe(true);
    expect(
      advanced
        .flatMap((group) => group.layers)
        .some((layer) => layer.layer_id === "terrain_mesh"),
    ).toBe(false);
  });
});
