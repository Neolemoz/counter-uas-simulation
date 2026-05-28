import type { MirrorEntity } from "./entityMarkers";
import { listOcclusionMarkers } from "./rtFictionalTerrain";
import type { TerrainLayerVisibility } from "./terrainLayers";
import { losCueSummary, nearestOcclusionTarget } from "./terrainCognition";
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
): string {
  return visibilityHubSummary(visibility, ctx);
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
  return labels;
}

export function sensorBlockVisible(terrainLayers: TerrainLayerVisibility): boolean {
  return terrainLayers.showSensorDomes || terrainLayers.showEnvironmentMarkers;
}
