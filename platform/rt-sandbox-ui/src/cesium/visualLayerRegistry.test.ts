import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { DEFAULT_TERRAIN_LAYERS } from "./terrainLayers";
import {
  anyVisibilityOverlayEnabled,
  CANONICAL_VISUAL_LAYER_REGISTRY,
  countActiveOverlayLayers,
  defaultVisibilityFromRegistry,
  groupLayersForUi,
  isOverlayBudgetExceeded,
  layersSortedByZOrder,
  performanceBudgetAdvisory,
  countVisibleLayers,
  registryBudgetSummaryLine,
  SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3,
  toggleableLayers,
  toTerrainLayerVisibility,
  validateVisualLayerRegistry,
  type VisualLayerRegistryV3,
} from "./visualLayerRegistry";

const REPO_ROOT = join(process.cwd(), "..", "..");
const FIXTURE_PATH = join(
  REPO_ROOT,
  "fixtures",
  "rt_visualization",
  "v3_layer_registry_example.json",
);

function loadRepoFixture(): VisualLayerRegistryV3 {
  return JSON.parse(readFileSync(FIXTURE_PATH, "utf-8")) as VisualLayerRegistryV3;
}

describe("visualLayerRegistry", () => {
  it("validates canonical registry", () => {
    expect(() => validateVisualLayerRegistry(CANONICAL_VISUAL_LAYER_REGISTRY)).not.toThrow();
  });

  it("validates repo fixture and matches canonical layer ids", () => {
    const fixture = loadRepoFixture();
    validateVisualLayerRegistry(fixture);
    const canonicalIds = CANONICAL_VISUAL_LAYER_REGISTRY.layers.map((l) => l.layer_id).sort();
    const fixtureIds = fixture.layers.map((l) => l.layer_id).sort();
    expect(fixtureIds).toEqual(canonicalIds);
    expect(fixture.schema).toBe(SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3);
  });

  it("layers have non-decreasing z_order when sorted", () => {
    const sorted = layersSortedByZOrder(CANONICAL_VISUAL_LAYER_REGISTRY);
    for (let i = 1; i < sorted.length; i++) {
      expect(sorted[i].z_order).toBeGreaterThanOrEqual(sorted[i - 1].z_order);
    }
  });

  it("default visibility matches frozen terrain defaults and panel chrome", () => {
    const defaults = defaultVisibilityFromRegistry();
    expect(toTerrainLayerVisibility(defaults)).toEqual(DEFAULT_TERRAIN_LAYERS);
    expect(defaults.showBounds).toBe(true);
    expect(defaults.showVerticalBounds).toBe(true);
    expect(defaults.showLabels).toBe(true);
  });

  it("preserves frozen default-off policy for optional layers", () => {
    const defaults = defaultVisibilityFromRegistry();
    expect(defaults.showContourOverlays).toBe(false);
    expect(defaults.showVegetationMarkers).toBe(false);
    expect(defaults.showEnvironmentMarkers).toBe(false);
    expect(defaults.showSensorDomes).toBe(false);
  });

  it("preserves frozen default-on for mesh, ridges, and entity_markers registry row", () => {
    const defaults = defaultVisibilityFromRegistry();
    expect(defaults.showTerrainMesh).toBe(true);
    expect(defaults.showRidgeOverlays).toBe(true);
    const entityMarkers = CANONICAL_VISUAL_LAYER_REGISTRY.layers.find(
      (l) => l.layer_id === "entity_markers",
    );
    expect(entityMarkers?.default_on).toBe(true);
  });

  it("requires disclaimers when cognition_kind is not none", () => {
    for (const layer of CANONICAL_VISUAL_LAYER_REGISTRY.layers) {
      if (layer.cognition_kind === "none") continue;
      expect(layer.disclaimer.trim().length).toBeGreaterThan(0);
    }
  });

  it("lists P1 visibility layers as toggleable with defaults off", () => {
    const p1Ids = ["visibility_wedge_v3", "horizon_hint_v3", "stacked_los_v3"];
    for (const id of p1Ids) {
      const layer = CANONICAL_VISUAL_LAYER_REGISTRY.layers.find((l) => l.layer_id === id);
      expect(layer).toBeDefined();
      expect(layer?.plat_phase).toBe("p1");
      expect(layer?.toggleable).toBe(true);
      expect(layer?.default_on).toBe(false);
    }
    const toggleIds = new Set(toggleableLayers(CANONICAL_VISUAL_LAYER_REGISTRY).map((l) => l.layer_id));
    for (const id of p1Ids) {
      expect(toggleIds.has(id)).toBe(true);
    }
    const defaults = defaultVisibilityFromRegistry();
    expect(defaults.showVisibilityWedge).toBe(false);
    expect(anyVisibilityOverlayEnabled(defaults)).toBe(false);
  });

  it("exposes performance budget caps from contract", () => {
    const budget = CANONICAL_VISUAL_LAYER_REGISTRY.performance_budget;
    expect(budget.max_active_overlay_layers).toBe(6);
    expect(budget.max_cesium_decor_entities).toBe(120);
    expect(budget.max_wedge_polylines_per_session).toBe(4);
  });

  it("groups toggleable layers for UI by cognition context", () => {
    const groups = groupLayersForUi(CANONICAL_VISUAL_LAYER_REGISTRY);
    expect(groups.length).toBeGreaterThanOrEqual(2);
    const terrain = groups.find((g) => g.groupId === "terrain_context");
    expect(terrain?.layers.some((l) => l.layer_id === "terrain_mesh")).toBe(true);
    const visibility = groups.find((g) => g.groupId === "visibility_context");
    expect(visibility?.layers.length).toBe(3);
  });

  it("counts active overlay layers for budget advisory", () => {
    const defaults = defaultVisibilityFromRegistry();
    expect(countActiveOverlayLayers(defaults)).toBe(2);
  });

  it("warns when overlay budget exceeded (advisory only)", () => {
    const v = {
      ...defaultVisibilityFromRegistry(),
      showContourOverlays: true,
      showVegetationMarkers: true,
      showEnvironmentMarkers: true,
      showSensorDomes: true,
      showVisibilityWedge: true,
    };
    expect(isOverlayBudgetExceeded(v)).toBe(true);
    expect(performanceBudgetAdvisory(v)).toMatch(/advisory/i);
  });

  it("counts visible toggleable layers", () => {
    const defaults = defaultVisibilityFromRegistry();
    const { on, total } = countVisibleLayers(defaults);
    expect(on).toBeGreaterThan(0);
    expect(total).toBeGreaterThan(on);
  });

  it("registry budget summary line is advisory", () => {
    const defaults = defaultVisibilityFromRegistry();
    expect(registryBudgetSummaryLine(defaults)).toMatch(/advisory/i);
    const heavy = {
      ...defaults,
      showContourOverlays: true,
      showVegetationMarkers: true,
      showEnvironmentMarkers: true,
      showSensorDomes: true,
      showVisibilityWedge: true,
    };
    expect(registryBudgetSummaryLine(heavy)).toMatch(/advisory/i);
    expect(registryBudgetSummaryLine(heavy)).toMatch(/overlays/);
  });
});
