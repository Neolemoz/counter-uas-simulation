import registryJson from "./fixtures/v3_layer_registry_v3.json";
import type { TerrainLayerVisibility } from "./terrainLayers";

export const SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3 = "rt_visual_layer_registry_v3";

export type CognitionKind = "terrain" | "visibility" | "sensor" | "marker" | "none";

export type CognitionGroupId =
  | "terrain_context"
  | "visibility_context"
  | "sensor_context"
  | "marker_context";

export type PlatPhase = "p0" | "p1";

export type LayerVisibilityKey =
  | "showTerrainMesh"
  | "showRidgeOverlays"
  | "showContourOverlays"
  | "showVegetationMarkers"
  | "showEnvironmentMarkers"
  | "showSensorDomes"
  | "showBounds"
  | "showVerticalBounds"
  | "showLabels"
  | "showVisibilityWedge"
  | "showHorizonHint"
  | "showStackedLos";

export interface VisualLayerPerformanceBudget {
  max_active_overlay_layers: number;
  max_cesium_decor_entities: number;
  max_wedge_polylines_per_session: number;
}

export interface VisualLayerDescriptor {
  layer_id: string;
  label: string;
  z_order: number;
  default_on: boolean;
  plat_phase: PlatPhase;
  toggleable: boolean;
  visibility_key?: LayerVisibilityKey;
  cognition_group: CognitionGroupId;
  module_anchor: string;
  cognition_kind: CognitionKind;
  disclaimer: string;
  mutual_exclusion_group?: string;
}

export interface VisualLayerRegistryV3 {
  schema: typeof SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3;
  performance_budget: VisualLayerPerformanceBudget;
  layers: VisualLayerDescriptor[];
}

export interface VisualLayerVisibility {
  showTerrainMesh: boolean;
  showRidgeOverlays: boolean;
  showContourOverlays: boolean;
  showVegetationMarkers: boolean;
  showEnvironmentMarkers: boolean;
  showSensorDomes: boolean;
  showBounds: boolean;
  showVerticalBounds: boolean;
  showLabels: boolean;
  showVisibilityWedge: boolean;
  showHorizonHint: boolean;
  showStackedLos: boolean;
}

export const COGNITION_GROUP_TITLES: Record<CognitionGroupId, string> = {
  terrain_context: "Terrain (explanatory)",
  visibility_context: "Visibility (heuristic)",
  sensor_context: "Sensor context (nominal)",
  marker_context: "Markers & bounds",
};

const COGNITION_KINDS: readonly CognitionKind[] = [
  "terrain",
  "visibility",
  "sensor",
  "marker",
  "none",
];

const LAYER_ID_TO_VISIBILITY_KEY: Partial<Record<string, LayerVisibilityKey>> = {
  terrain_mesh: "showTerrainMesh",
  ridge_overlays: "showRidgeOverlays",
  contour_overlays: "showContourOverlays",
  vegetation_markers: "showVegetationMarkers",
  occlusion_markers: "showEnvironmentMarkers",
  sensor_domes: "showSensorDomes",
  bounds_overlay: "showBounds",
  bounds_vertical: "showVerticalBounds",
  entity_labels: "showLabels",
  visibility_wedge_v3: "showVisibilityWedge",
  horizon_hint_v3: "showHorizonHint",
  stacked_los_v3: "showStackedLos",
};

export const CANONICAL_VISUAL_LAYER_REGISTRY =
  registryJson as VisualLayerRegistryV3;

export class VisualLayerRegistryValidationError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "VisualLayerRegistryValidationError";
  }
}

export function validateVisualLayerRegistry(reg: VisualLayerRegistryV3): void {
  if (reg.schema !== SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3) {
    throw new VisualLayerRegistryValidationError(
      `schema must be ${SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3}`,
    );
  }

  const budget = reg.performance_budget;
  if (
    typeof budget.max_active_overlay_layers !== "number" ||
    typeof budget.max_cesium_decor_entities !== "number" ||
    typeof budget.max_wedge_polylines_per_session !== "number"
  ) {
    throw new VisualLayerRegistryValidationError("performance_budget caps must be numbers");
  }

  const seen = new Set<string>();
  for (const layer of reg.layers) {
    if (seen.has(layer.layer_id)) {
      throw new VisualLayerRegistryValidationError(`duplicate layer_id: ${layer.layer_id}`);
    }
    seen.add(layer.layer_id);

    if (!COGNITION_KINDS.includes(layer.cognition_kind)) {
      throw new VisualLayerRegistryValidationError(
        `invalid cognition_kind on ${layer.layer_id}`,
      );
    }

    if (layer.cognition_kind !== "none" && !layer.disclaimer.trim()) {
      throw new VisualLayerRegistryValidationError(
        `disclaimer required when cognition_kind is not none (${layer.layer_id})`,
      );
    }

    if (layer.toggleable) {
      const key = layer.visibility_key ?? layerIdToVisibilityKey(layer.layer_id);
      if (!key) {
        throw new VisualLayerRegistryValidationError(
          `toggleable layer missing visibility_key: ${layer.layer_id}`,
        );
      }
    }
  }
}

export function layersSortedByZOrder(reg: VisualLayerRegistryV3): VisualLayerDescriptor[] {
  return [...reg.layers].sort((a, b) => a.z_order - b.z_order);
}

export function layerIdToVisibilityKey(layerId: string): LayerVisibilityKey | undefined {
  return LAYER_ID_TO_VISIBILITY_KEY[layerId];
}

export function defaultVisibilityFromRegistry(
  reg: VisualLayerRegistryV3 = CANONICAL_VISUAL_LAYER_REGISTRY,
): VisualLayerVisibility {
  const visibility: VisualLayerVisibility = {
    showTerrainMesh: false,
    showRidgeOverlays: false,
    showContourOverlays: false,
    showVegetationMarkers: false,
    showEnvironmentMarkers: false,
    showSensorDomes: false,
    showBounds: false,
    showVerticalBounds: false,
    showLabels: false,
    showVisibilityWedge: false,
    showHorizonHint: false,
    showStackedLos: false,
  };

  for (const layer of reg.layers) {
    const key = layer.visibility_key ?? layerIdToVisibilityKey(layer.layer_id);
    if (!key) continue;
    visibility[key] = layer.default_on;
  }

  return visibility;
}

export function toTerrainLayerVisibility(v: VisualLayerVisibility): TerrainLayerVisibility {
  return {
    showTerrainMesh: v.showTerrainMesh,
    showRidgeOverlays: v.showRidgeOverlays,
    showContourOverlays: v.showContourOverlays,
    showVegetationMarkers: v.showVegetationMarkers,
    showEnvironmentMarkers: v.showEnvironmentMarkers,
    showSensorDomes: v.showSensorDomes,
  };
}

export interface UiLayerGroup {
  groupId: CognitionGroupId;
  title: string;
  layers: VisualLayerDescriptor[];
}

export function toggleableLayers(reg: VisualLayerRegistryV3): VisualLayerDescriptor[] {
  return reg.layers.filter((l) => l.toggleable);
}

export function groupLayersForUi(reg: VisualLayerRegistryV3): UiLayerGroup[] {
  const order: CognitionGroupId[] = [
    "terrain_context",
    "visibility_context",
    "sensor_context",
    "marker_context",
  ];
  const toggleable = toggleableLayers(reg);
  return order
    .map((groupId) => ({
      groupId,
      title: COGNITION_GROUP_TITLES[groupId],
      layers: toggleable.filter((l) => l.cognition_group === groupId),
    }))
    .filter((g) => g.layers.length > 0);
}

export function isLayerVisible(
  visibility: VisualLayerVisibility,
  layer: VisualLayerDescriptor,
): boolean {
  const key = layer.visibility_key ?? layerIdToVisibilityKey(layer.layer_id);
  if (!key) return layer.default_on;
  return visibility[key];
}

export function toggleLayerVisibility(
  visibility: VisualLayerVisibility,
  layerId: string,
  reg: VisualLayerRegistryV3 = CANONICAL_VISUAL_LAYER_REGISTRY,
): VisualLayerVisibility {
  const key = layerIdToVisibilityKey(layerId);
  if (!key) return visibility;
  const layer = reg.layers.find((l) => l.layer_id === layerId);
  if (!layer?.toggleable) return visibility;
  return { ...visibility, [key]: !visibility[key] };
}

const OVERLAY_COUNT_KEYS: LayerVisibilityKey[] = [
  "showTerrainMesh",
  "showRidgeOverlays",
  "showContourOverlays",
  "showVegetationMarkers",
  "showEnvironmentMarkers",
  "showSensorDomes",
  "showVisibilityWedge",
];

export function countActiveOverlayLayers(
  visibility: VisualLayerVisibility,
  _reg: VisualLayerRegistryV3 = CANONICAL_VISUAL_LAYER_REGISTRY,
): number {
  return OVERLAY_COUNT_KEYS.filter((k) => visibility[k]).length;
}

export function isOverlayBudgetExceeded(
  visibility: VisualLayerVisibility,
  reg: VisualLayerRegistryV3 = CANONICAL_VISUAL_LAYER_REGISTRY,
): boolean {
  return countActiveOverlayLayers(visibility, reg) > reg.performance_budget.max_active_overlay_layers;
}

export function anyVisibilityOverlayEnabled(visibility: VisualLayerVisibility): boolean {
  return (
    visibility.showVisibilityWedge ||
    visibility.showHorizonHint ||
    visibility.showStackedLos
  );
}

export function performanceBudgetAdvisory(
  visibility: VisualLayerVisibility,
  reg: VisualLayerRegistryV3 = CANONICAL_VISUAL_LAYER_REGISTRY,
): string | null {
  const active = countActiveOverlayLayers(visibility, reg);
  const cap = reg.performance_budget.max_active_overlay_layers;
  if (active <= cap) return null;
  return `Overlay budget: ${active}/${cap} active layers (advisory — not enforced)`;
}

export function countVisibleLayers(
  visibility: VisualLayerVisibility,
  reg: VisualLayerRegistryV3 = CANONICAL_VISUAL_LAYER_REGISTRY,
): { on: number; total: number } {
  const toggleable = toggleableLayers(reg);
  const on = toggleable.filter((layer) => isLayerVisible(visibility, layer)).length;
  return { on, total: toggleable.length };
}

export function registryBudgetSummaryLine(
  visibility: VisualLayerVisibility,
  reg: VisualLayerRegistryV3 = CANONICAL_VISUAL_LAYER_REGISTRY,
): string {
  const { on, total } = countVisibleLayers(visibility, reg);
  const overlays = countActiveOverlayLayers(visibility, reg);
  const cap = reg.performance_budget.max_active_overlay_layers;
  const advisory = performanceBudgetAdvisory(visibility, reg);
  const base = `Layers ${on}/${total} on · overlays ${overlays}/${cap}`;
  return advisory ? `${base} · ${advisory}` : `${base} (advisory)`;
}

validateVisualLayerRegistry(CANONICAL_VISUAL_LAYER_REGISTRY);
