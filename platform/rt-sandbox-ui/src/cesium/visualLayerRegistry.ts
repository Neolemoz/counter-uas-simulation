import registryJson from "./fixtures/v3_layer_registry_v3.json";
import type { TerrainLayerVisibility } from "./terrainLayers";

export const SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3 = "rt_visual_layer_registry_v3";
export const SCHEMA_RT_VISUAL_LAYER_REGISTRY_V4 = "rt_visual_layer_registry_v4";

export type CognitionKind =
  | "terrain"
  | "visibility"
  | "sensor"
  | "marker"
  | "density"
  | "comparison"
  | "none";

export type CognitionGroupId =
  | "terrain_context"
  | "visibility_context"
  | "sensor_context"
  | "marker_context"
  | "density_context"
  | "comparison_context";

export type DensityGroupId =
  | "authority_context"
  | "entity_context"
  | "terrain_context"
  | "visibility_context"
  | "diagnostic_context"
  | "decorative_context";

export type VisualComparisonRole =
  | "selected"
  | "comparison"
  | "background";

export type PlatPhase = "p0" | "p1" | "v4_p0" | "v4_p1";

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
  | "showStackedLos"
  | "showDensityWarnings"
  | "showLayerBudgetSummary"
  | "showSessionContrast"
  | "showComparisonGhosts"
  | "showVisibilityCorridorV4"
  | "showOcclusionBandsV4"
  | "showTerrainRelationLabelsV4"
  | "showCompareEmphasisV4"
  | "showTacticalPredictedPath"
  | "showTacticalInterceptPoint"
  | "showTacticalThreatCorridor"
  | "showTacticalRankingCues"
  | "showTacticalTimingLabels"
  | "showTacticalSelectionEmphasis"
  | "showTacticalCompareOverlay"
  | "showRuntimeCoverageCells";

export interface VisualLayerPerformanceBudget {
  max_active_overlay_layers: number;
  max_cesium_decor_entities: number;
  max_wedge_polylines_per_session: number;
  max_density_warning_groups?: number;
  max_session_comparison_rows?: number;
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
  density_group?: DensityGroupId;
  comparison_role?: VisualComparisonRole;
  display_only?: boolean;
}

export interface VisualLayerRegistryV4 {
  schema:
    | typeof SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3
    | typeof SCHEMA_RT_VISUAL_LAYER_REGISTRY_V4;
  performance_budget: VisualLayerPerformanceBudget;
  layers: VisualLayerDescriptor[];
}

export type VisualLayerRegistryV3 = VisualLayerRegistryV4;

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
  showDensityWarnings: boolean;
  showLayerBudgetSummary: boolean;
  showSessionContrast: boolean;
  showComparisonGhosts: boolean;
  showVisibilityCorridorV4: boolean;
  showOcclusionBandsV4: boolean;
  showTerrainRelationLabelsV4: boolean;
  showCompareEmphasisV4: boolean;
  showTacticalPredictedPath: boolean;
  showTacticalInterceptPoint: boolean;
  showTacticalThreatCorridor: boolean;
  showTacticalRankingCues: boolean;
  showTacticalTimingLabels: boolean;
  showTacticalSelectionEmphasis: boolean;
  showTacticalCompareOverlay: boolean;
  showRuntimeCoverageCells: boolean;
}

export const COGNITION_GROUP_TITLES: Record<CognitionGroupId, string> = {
  terrain_context: "Terrain (explanatory)",
  visibility_context: "Visibility (heuristic)",
  sensor_context: "Sensor context (nominal)",
  marker_context: "Markers & bounds",
  density_context: "Density controls",
  comparison_context: "Session compare (visual only)",
};

const COGNITION_KINDS: readonly CognitionKind[] = [
  "terrain",
  "visibility",
  "sensor",
  "marker",
  "density",
  "comparison",
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
  density_warnings_v4: "showDensityWarnings",
  layer_budget_summary_v4: "showLayerBudgetSummary",
  session_contrast_v4: "showSessionContrast",
  comparison_ghosts_v4: "showComparisonGhosts",
  visibility_corridor_v4: "showVisibilityCorridorV4",
  occlusion_bands_v4: "showOcclusionBandsV4",
  terrain_relation_labels_v4: "showTerrainRelationLabelsV4",
  compare_emphasis_v4: "showCompareEmphasisV4",
  tactical_predicted_path: "showTacticalPredictedPath",
  tactical_intercept_point: "showTacticalInterceptPoint",
  tactical_threat_corridor: "showTacticalThreatCorridor",
  tactical_ranking_cues: "showTacticalRankingCues",
  tactical_timing_labels: "showTacticalTimingLabels",
  tactical_selection_emphasis: "showTacticalSelectionEmphasis",
  tactical_compare_overlay: "showTacticalCompareOverlay",
  runtime_coverage_cells: "showRuntimeCoverageCells",
};

const V4_P0_LAYERS: VisualLayerDescriptor[] = [
  {
    layer_id: "density_warnings_v4",
    label: "Density warnings",
    z_order: 80,
    default_on: false,
    plat_phase: "v4_p0",
    toggleable: true,
    visibility_key: "showDensityWarnings",
    cognition_group: "density_context",
    module_anchor: "visualDensityPolicy",
    cognition_kind: "density",
    density_group: "diagnostic_context",
    display_only: true,
    disclaimer: "Warn-only density indicator — does not enforce or command",
  },
  {
    layer_id: "layer_budget_summary_v4",
    label: "Budget summary",
    z_order: 81,
    default_on: false,
    plat_phase: "v4_p0",
    toggleable: true,
    visibility_key: "showLayerBudgetSummary",
    cognition_group: "density_context",
    module_anchor: "LayerDensitySummary",
    cognition_kind: "density",
    density_group: "diagnostic_context",
    display_only: true,
    disclaimer: "Layer budget summary — explanatory display only",
  },
  {
    layer_id: "session_contrast_v4",
    label: "Session contrast",
    z_order: 82,
    default_on: true,
    plat_phase: "v4_p0",
    toggleable: true,
    visibility_key: "showSessionContrast",
    cognition_group: "comparison_context",
    module_anchor: "SessionComparisonCognitionStrip",
    cognition_kind: "comparison",
    density_group: "entity_context",
    comparison_role: "selected",
    display_only: true,
    disclaimer: "Session contrast is visual only — command target remains selected session",
  },
  {
    layer_id: "comparison_ghosts_v4",
    label: "Compare ghosts",
    z_order: 83,
    default_on: false,
    plat_phase: "v4_p0",
    toggleable: true,
    visibility_key: "showComparisonGhosts",
    cognition_group: "comparison_context",
    module_anchor: "SessionComparisonCognitionStrip",
    cognition_kind: "comparison",
    density_group: "entity_context",
    comparison_role: "comparison",
    display_only: true,
    disclaimer: "Comparison ghosts are explanatory only — no cross-session commands",
  },
];

const V4_P1_LAYERS: VisualLayerDescriptor[] = [
  {
    layer_id: "visibility_corridor_v4",
    label: "Visibility corridor",
    z_order: 84,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showVisibilityCorridorV4",
    cognition_group: "visibility_context",
    module_anchor: "visibilityOverlayV4",
    cognition_kind: "visibility",
    density_group: "visibility_context",
    display_only: true,
    disclaimer: "Visibility corridor is a heuristic display cue - not coverage authority",
  },
  {
    layer_id: "occlusion_bands_v4",
    label: "Occlusion bands",
    z_order: 85,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showOcclusionBandsV4",
    cognition_group: "visibility_context",
    module_anchor: "visibilityOverlayV4",
    cognition_kind: "visibility",
    density_group: "visibility_context",
    display_only: true,
    disclaimer: "Occlusion bands are fictional-terrain hints - no simulation state changes",
  },
  {
    layer_id: "terrain_relation_labels_v4",
    label: "Terrain labels",
    z_order: 86,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showTerrainRelationLabelsV4",
    cognition_group: "visibility_context",
    module_anchor: "visibilityOverlayV4",
    cognition_kind: "visibility",
    density_group: "visibility_context",
    display_only: true,
    disclaimer: "Terrain relation labels are explanatory only",
  },
  {
    layer_id: "compare_emphasis_v4",
    label: "Compare emphasis",
    z_order: 87,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showCompareEmphasisV4",
    cognition_group: "comparison_context",
    module_anchor: "SessionComparisonCognitionStrip",
    cognition_kind: "comparison",
    density_group: "entity_context",
    comparison_role: "background",
    display_only: true,
    disclaimer: "Compare emphasis dims background sessions visually only - selected session remains commandable",
  },
];

/** Unified ±7 km tactical overlay governance — display-only, no command or engagement authority. */
export const TACTICAL_VISUALIZATION_GOVERNANCE_COPY =
  "Tactical visualization overlays are display-only — no command authority, no intercept assignment authority, and no autonomous engagement authority.";

const TACTICAL_LAYERS: VisualLayerDescriptor[] = [
  {
    layer_id: "tactical_predicted_path",
    label: "Tactical path",
    z_order: 88,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showTacticalPredictedPath",
    cognition_group: "marker_context",
    module_anchor: "tacticalTrajectoryLayer",
    cognition_kind: "marker",
    density_group: "entity_context",
    display_only: true,
    disclaimer:
      "Tactical predicted path is display-only for the unified ±7 km world — not command authority or autonomous engagement",
  },
  {
    layer_id: "tactical_intercept_point",
    label: "Solution point",
    z_order: 89,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showTacticalInterceptPoint",
    cognition_group: "marker_context",
    module_anchor: "tacticalTrajectoryLayer",
    cognition_kind: "marker",
    density_group: "entity_context",
    display_only: true,
    disclaimer:
      "Solution point marker is explanatory telemetry only — not assignment, command, or autonomous engagement authority",
  },
  {
    layer_id: "tactical_threat_corridor",
    label: "Threat corridor",
    z_order: 87,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showTacticalThreatCorridor",
    cognition_group: "marker_context",
    module_anchor: "tacticalCorridorLayer",
    cognition_kind: "marker",
    density_group: "entity_context",
    display_only: true,
    disclaimer:
      "Threat corridor is display-only attacker-to-solution emphasis — not weapon engagement or autonomous intercept authority",
  },
  {
    layer_id: "tactical_ranking_cues",
    label: "Ranking cues",
    z_order: 86,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showTacticalRankingCues",
    cognition_group: "marker_context",
    module_anchor: "tacticalRankingCueLayer",
    cognition_kind: "marker",
    density_group: "entity_context",
    display_only: true,
    disclaimer:
      "Ranking cues are explanatory display only — not command authority, operational prioritization, or autonomous engagement",
  },
  {
    layer_id: "tactical_timing_labels",
    label: "Timing labels",
    z_order: 90,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showTacticalTimingLabels",
    cognition_group: "marker_context",
    module_anchor: "tacticalTrajectoryLayer",
    cognition_kind: "marker",
    density_group: "entity_context",
    display_only: true,
    disclaimer:
      "TTI/ETA labels are explanatory tactical telemetry — not command authority or autonomous engagement timing",
  },
  {
    layer_id: "tactical_selection_emphasis",
    label: "Tactical target",
    z_order: 91,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showTacticalSelectionEmphasis",
    cognition_group: "marker_context",
    module_anchor: "tacticalSelectionEmphasisLayer",
    cognition_kind: "marker",
    density_group: "entity_context",
    display_only: true,
    disclaimer:
      "Tactical target emphasis is visual only — edit selection remains command priority; no autonomous engagement",
  },
  {
    layer_id: "tactical_compare_overlay",
    label: "Tactical compare",
    z_order: 84,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showTacticalCompareOverlay",
    cognition_group: "comparison_context",
    module_anchor: "tacticalCompareOverlay",
    cognition_kind: "comparison",
    density_group: "entity_context",
    comparison_role: "comparison",
    display_only: true,
    disclaimer:
      "Tactical compare ghosts are display-only — not command, outcome, or autonomous engagement authority",
  },
];

const RUNTIME_COVERAGE_LAYERS: VisualLayerDescriptor[] = [
  {
    layer_id: "runtime_coverage_cells",
    label: "Runtime coverage cells",
    z_order: 51,
    default_on: false,
    plat_phase: "v4_p1",
    toggleable: true,
    visibility_key: "showRuntimeCoverageCells",
    cognition_group: "sensor_context",
    module_anchor: "runtimeCoverageLayer",
    cognition_kind: "sensor",
    density_group: "visibility_context",
    display_only: true,
    disclaimer:
      "Runtime coverage cells are heuristic 2D geometry only — not sensor truth or detection probability",
  },
];

export const CANONICAL_VISUAL_LAYER_REGISTRY: VisualLayerRegistryV4 = {
  ...(registryJson as VisualLayerRegistryV3),
  schema: SCHEMA_RT_VISUAL_LAYER_REGISTRY_V4,
  performance_budget: {
    ...(registryJson as VisualLayerRegistryV3).performance_budget,
    max_density_warning_groups: 5,
    max_session_comparison_rows: 2,
  },
  layers: [
    ...(registryJson as VisualLayerRegistryV3).layers,
    ...V4_P0_LAYERS,
    ...V4_P1_LAYERS,
    ...TACTICAL_LAYERS,
    ...RUNTIME_COVERAGE_LAYERS,
  ],
};

export class VisualLayerRegistryValidationError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "VisualLayerRegistryValidationError";
  }
}

export function validateVisualLayerRegistry(reg: VisualLayerRegistryV3): void {
  if (
    reg.schema !== SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3 &&
    reg.schema !== SCHEMA_RT_VISUAL_LAYER_REGISTRY_V4
  ) {
    throw new VisualLayerRegistryValidationError(
      `schema must be ${SCHEMA_RT_VISUAL_LAYER_REGISTRY_V3} or ${SCHEMA_RT_VISUAL_LAYER_REGISTRY_V4}`,
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
    showDensityWarnings: false,
    showLayerBudgetSummary: false,
    showSessionContrast: false,
    showComparisonGhosts: false,
    showVisibilityCorridorV4: false,
    showOcclusionBandsV4: false,
    showTerrainRelationLabelsV4: false,
    showCompareEmphasisV4: false,
    showTacticalPredictedPath: false,
    showTacticalInterceptPoint: false,
    showTacticalThreatCorridor: false,
    showTacticalRankingCues: false,
    showTacticalTimingLabels: false,
    showTacticalSelectionEmphasis: false,
    showTacticalCompareOverlay: false,
    showRuntimeCoverageCells: false,
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
    "density_context",
    "comparison_context",
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
  "showVisibilityCorridorV4",
  "showOcclusionBandsV4",
  "showTerrainRelationLabelsV4",
  "showCompareEmphasisV4",
  "showTacticalPredictedPath",
  "showTacticalInterceptPoint",
  "showTacticalThreatCorridor",
  "showTacticalRankingCues",
  "showTacticalTimingLabels",
  "showTacticalSelectionEmphasis",
  "showTacticalCompareOverlay",
  "showRuntimeCoverageCells",
];

const DENSITY_CONTROL_KEYS: LayerVisibilityKey[] = [
  "showDensityWarnings",
  "showLayerBudgetSummary",
  "showSessionContrast",
  "showComparisonGhosts",
  "showCompareEmphasisV4",
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
    visibility.showStackedLos ||
    visibility.showVisibilityCorridorV4 ||
    visibility.showOcclusionBandsV4 ||
    visibility.showTerrainRelationLabelsV4
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

export interface DensityBudgetSummary {
  activeOverlayLayers: number;
  overlayCap: number;
  activeDensityControls: number;
  densityControlCount: number;
  densityWarningsEnabled: boolean;
  layerBudgetSummaryEnabled: boolean;
  exceeded: boolean;
  line: string;
}

export function densityControlLayers(
  reg: VisualLayerRegistryV3 = CANONICAL_VISUAL_LAYER_REGISTRY,
): VisualLayerDescriptor[] {
  return reg.layers.filter((l) => l.cognition_group === "density_context");
}

export function countActiveDensityControls(
  visibility: VisualLayerVisibility,
): number {
  return DENSITY_CONTROL_KEYS.filter((k) => visibility[k]).length;
}

export function densityBudgetSummary(
  visibility: VisualLayerVisibility,
  reg: VisualLayerRegistryV3 = CANONICAL_VISUAL_LAYER_REGISTRY,
): DensityBudgetSummary {
  const activeOverlayLayers = countActiveOverlayLayers(visibility, reg);
  const overlayCap = reg.performance_budget.max_active_overlay_layers;
  const activeDensityControls = countActiveDensityControls(visibility);
  const densityControlCount = densityControlLayers(reg).length;
  const exceeded = activeOverlayLayers > overlayCap;
  const status = exceeded && visibility.showDensityWarnings ? "warn-only" : "advisory";
  return {
    activeOverlayLayers,
    overlayCap,
    activeDensityControls,
    densityControlCount,
    densityWarningsEnabled: visibility.showDensityWarnings,
    layerBudgetSummaryEnabled: visibility.showLayerBudgetSummary,
    exceeded,
    line: `Density ${activeDensityControls}/${densityControlCount} controls · overlays ${activeOverlayLayers}/${overlayCap} (${status})`,
  };
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
  const density = densityBudgetSummary(visibility, reg);
  const base = `Layers ${on}/${total} on · overlays ${overlays}/${cap}`;
  if (!visibility.showLayerBudgetSummary) {
    return `${base} · budget summary off (advisory)`;
  }
  return advisory ? `${base} · ${advisory}` : `${base} · ${density.line}`;
}

validateVisualLayerRegistry(CANONICAL_VISUAL_LAYER_REGISTRY);
