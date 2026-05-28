import type { MirrorEntity } from "./entityMarkers";
import { listOcclusionMarkers } from "./rtFictionalTerrain";
import type { TerrainLayerVisibility } from "./terrainLayers";
import { losCueSummary, nearestOcclusionTarget } from "./terrainCognition";
import {
  deriveVisibilityOverlayV4Hints,
  visibilityOverlayV4SummaryLine,
} from "./visibilityOverlayV4";
import type { VisualLayerVisibility } from "./visualLayerRegistry";
import {
  anyVisibilityOverlayEnabled,
  performanceBudgetAdvisory,
} from "./visualLayerRegistry";

export interface VisibilityHubContext {
  selectedEntity: MirrorEntity | null;
  entities: MirrorEntity[];
}

export function visibilityHubSummary(
  visibility: VisualLayerVisibility,
  ctx: VisibilityHubContext,
): string {
  const parts: string[] = [];
  if (visibility.showVisibilityWedge) {
    parts.push("Wedge: heuristic ±30°");
  }
  if (visibility.showHorizonHint) {
    parts.push("Horizon: fictional bounds cue");
  }
  if (visibility.showStackedLos) {
    parts.push("Stacked LOS: on");
  }
  if (visibility.showVisibilityCorridorV4) {
    parts.push("V4 corridor: heuristic");
  }
  if (visibility.showOcclusionBandsV4) {
    parts.push("V4 occlusion bands: warn-only");
  }
  if (visibility.showTerrainRelationLabelsV4) {
    parts.push("V4 terrain labels: explanatory");
  }
  if (parts.length === 0) {
    return "Visibility overlays off — heuristic wedge/horizon/LOS available via toggles";
  }
  if (ctx.selectedEntity) {
    const occ = nearestOcclusionTarget(ctx.selectedEntity, ctx.entities);
    if (occ) {
      parts.push(`LOS ${occ.status} (explanatory)`);
    }
  }
  return parts.join(" · ");
}

export function sensorContextHubLine(
  terrainLayers: TerrainLayerVisibility,
  entities: MirrorEntity[],
): string | null {
  if (!terrainLayers.showSensorDomes && !terrainLayers.showEnvironmentMarkers) {
    return null;
  }
  const radarCount = entities.filter((e) => e.entity_type === "radar").length;
  const domeCount = terrainLayers.showSensorDomes ? radarCount : 0;
  const occlusionCount = terrainLayers.showEnvironmentMarkers
    ? listOcclusionMarkers().length
    : 0;
  return `Sensor context: ${domeCount} nominal domes · ${occlusionCount} occlusion markers (explanatory)`;
}

export function visibilityStripSummary(
  visibility: VisualLayerVisibility,
  ctx: VisibilityHubContext,
  terrainLayers?: TerrainLayerVisibility,
): string {
  const base = visibilityHubSummary(visibility, ctx);
  if (!terrainLayers) return base;
  const v4 = deriveVisibilityOverlayV4Hints({
    visibility,
    selected: ctx.selectedEntity,
    entities: ctx.entities,
    terrainLayers,
  });
  if (v4.length === 0) return base;
  return `${base} · ${visibilityOverlayV4SummaryLine(v4)}`;
}

export function visibilityLosLine(
  selected: MirrorEntity | null,
  entities: MirrorEntity[],
  showStackedLos: boolean,
): string | null {
  if (!showStackedLos || !selected) return null;
  const occ = nearestOcclusionTarget(selected, entities);
  if (!occ) return null;
  return `${losCueSummary(selected, occ.target)} (explanatory)`;
}

export function visibilityBudgetLine(visibility: VisualLayerVisibility): string | null {
  return performanceBudgetAdvisory(visibility);
}

export function anyVisibilityCognitionActive(visibility: VisualLayerVisibility): boolean {
  return anyVisibilityOverlayEnabled(visibility);
}

export function visibilityActiveLabels(visibility: VisualLayerVisibility): string[] {
  const labels: string[] = [];
  if (visibility.showVisibilityWedge) labels.push("wedge");
  if (visibility.showHorizonHint) labels.push("horizon");
  if (visibility.showStackedLos) labels.push("stacked LOS");
  if (visibility.showVisibilityCorridorV4) labels.push("V4 corridor");
  if (visibility.showOcclusionBandsV4) labels.push("V4 occlusion bands");
  if (visibility.showTerrainRelationLabelsV4) labels.push("V4 terrain labels");
  return labels;
}

export function sensorBlockVisible(terrainLayers: TerrainLayerVisibility): boolean {
  return terrainLayers.showSensorDomes || terrainLayers.showEnvironmentMarkers;
}
