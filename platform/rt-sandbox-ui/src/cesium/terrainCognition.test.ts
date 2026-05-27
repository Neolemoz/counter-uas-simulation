import { describe, expect, it } from "vitest";
import type { MirrorEntity } from "./entityMarkers";
import { DEFAULT_TERRAIN_LAYERS } from "./terrainLayers";
import {
  entityTerrainRelation,
  losCueSummary,
  occlusionHeuristic,
  sensorDomeContext,
  terrainHubSummary,
  visibilityHint,
} from "./terrainCognition";

describe("terrainCognition", () => {
  it("builds entity terrain relation", () => {
    const rel = entityTerrainRelation(0, 0, 25);
    expect(rel.registry_z_m).toBe(25);
    expect(rel.terrain_m).toBeGreaterThanOrEqual(0);
    expect(rel).toHaveProperty("contour_level_m");
  });

  it("classifies occlusion heuristic", () => {
    const a: MirrorEntity = {
      entity_id: "a",
      entity_type: "radar",
      pose: { x: 0, y: 0, z: 30 },
    };
    const b: MirrorEntity = {
      entity_id: "b",
      entity_type: "drone",
      pose: { x: 200, y: 0, z: 5 },
    };
    const status = occlusionHeuristic(a, b);
    expect(["clear", "terrain_blocked", "marker_occluded"]).toContain(status);
  });

  it("summarizes sensor dome context for radar", () => {
    const radar: MirrorEntity = {
      entity_id: "r1",
      entity_type: "radar",
      pose: { x: 0, y: 0, z: 10 },
    };
    const ctx = sensorDomeContext(radar, [radar]);
    expect(ctx.radius_m).toBe(200);
    expect(ctx.note).toContain("not sensor coverage");
    expect(ctx.context_label).toContain("explanatory");
  });

  it("builds LOS cue summary", () => {
    const a: MirrorEntity = {
      entity_id: "a",
      entity_type: "radar",
      pose: { x: 0, y: 0, z: 30 },
    };
    const b: MirrorEntity = {
      entity_id: "b",
      entity_type: "drone",
      pose: { x: 50, y: 0, z: 5 },
    };
    expect(losCueSummary(a, b)).toContain("LOS heuristic");
  });

  it("provides visibility hint", () => {
    const a: MirrorEntity = {
      entity_id: "a",
      entity_type: "radar",
      pose: { x: 0, y: 0, z: 30 },
    };
    const b: MirrorEntity = {
      entity_id: "b",
      entity_type: "drone",
      pose: { x: 80, y: 0, z: 5 },
    };
    expect(visibilityHint(a, [a, b])).toMatch(/heuristic/i);
  });

  it("summarizes hub with active layers", () => {
    const summary = terrainHubSummary(true, {
      ...DEFAULT_TERRAIN_LAYERS,
      showContourOverlays: true,
    });
    expect(summary).toContain("contours");
    expect(summary).toContain("heuristic");
  });
});
