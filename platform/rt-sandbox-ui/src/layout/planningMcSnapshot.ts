/**
 * UI-local Planning snapshot export for future Planning -> Monte Carlo linkage.
 * This module does not prepare jobs, execute MC, or touch runtime/bridge state.
 */

import {
  PLANNING_RADAR_PRESETS,
  type PlanningPolygonState,
  type PlanningRadarPresetId,
  type PlanningRadarSite,
  type PlanningRadarState,
  type PlanningVertex,
} from "@/cesium/planningDrawing";
import type { PlanningCoverageAnalysis } from "@/cesium/planningCoverageAnalysis";
import type { CesiumTerrainProviderMode } from "@/cesium/terrainProviderConfig";
import type { PlanningLocationPresetId } from "@/cesium/planningLocations";
import {
  planningExtentMetadataForExport,
  unifiedPlanningWorld,
  type PlanningExtent,
} from "@/cesium/planningWorld";
import { WORLD_BOUNDS } from "@/world/bounds";
import { sha256Hex16 } from "@/experiment/sha256Hex";
import { exportJson, copyTextToClipboard, type ClipboardResult } from "./layoutMcHandoff";

export const PLANNING_MC_SNAPSHOT_SCHEMA_VERSION = "rt_planning_mc_snapshot_v1" as const;

export type PlanningMcSnapshotVertex = {
  x: number;
  y: number;
};

export type PlanningMcSnapshotRadarSite = {
  id: string;
  position: PlanningMcSnapshotVertex;
  radar_type: string;
  radar_preset: PlanningRadarPresetId | null;
  detection_range_m: number;
};

export type PlanningMcSnapshotAnalyticsSummary = {
  coverage_percent: number;
  overlap_percent: number;
  redundancy_percent: number;
  blind_spot_summary: string;
  recommendation_summary: {
    suggested_radar: string | null;
    suggested_position: PlanningMcSnapshotVertex | null;
    reason: string | null;
  };
};

export type PlanningMcSnapshotPresentation = {
  terrain_mode: CesiumTerrainProviderMode;
  selected_location_preset: PlanningLocationPresetId;
};

export type PlanningMcSnapshotExtent = PlanningExtent;

export type PlanningMcSnapshotWorldBoundsM = {
  x_min_m: number;
  x_max_m: number;
  y_min_m: number;
  y_max_m: number;
  z_min_m: number;
  z_max_m: number;
};

export type PlanningMcSnapshotProvenance = {
  source_layout_id?: string;
  source_geometry_id?: string;
  authority: "rt_planning_ui";
  origin: "platform/rt-sandbox-ui";
  ui_flow: "planning_mode";
};

export type PlanningMcSnapshotV1 = {
  schema_version: typeof PLANNING_MC_SNAPSHOT_SCHEMA_VERSION;
  planning_snapshot_id: string;
  planning_geometry_id: string;
  created_utc: string;
  polygon: {
    defense_area_vertices: PlanningMcSnapshotVertex[];
  };
  radars: {
    radar_sites: PlanningMcSnapshotRadarSite[];
  };
  analytics_summary: PlanningMcSnapshotAnalyticsSummary;
  planning_extent: PlanningMcSnapshotExtent;
  world_bounds_m?: PlanningMcSnapshotWorldBoundsM;
  presentation: PlanningMcSnapshotPresentation;
  provenance: PlanningMcSnapshotProvenance;
};

export function worldBoundsSnapshotMetadata(): PlanningMcSnapshotWorldBoundsM {
  return {
    x_min_m: WORLD_BOUNDS.x.min,
    x_max_m: WORLD_BOUNDS.x.max,
    y_min_m: WORLD_BOUNDS.y.min,
    y_max_m: WORLD_BOUNDS.y.max,
    z_min_m: WORLD_BOUNDS.z.min,
    z_max_m: WORLD_BOUNDS.z.max,
  };
}

export type BuildPlanningMcSnapshotOptions = {
  createdUtc?: string;
  terrainMode: CesiumTerrainProviderMode;
  selectedLocationPreset: PlanningLocationPresetId;
  planningExtent?: PlanningExtent;
  sourceLayoutId?: string;
  sourceGeometryId?: string;
};

function utcNow(): string {
  return new Date().toISOString().replace(/\.\d{3}Z$/, "Z");
}

function stableStringify(value: unknown): string {
  if (value === null || typeof value !== "object") return JSON.stringify(value);
  if (Array.isArray(value)) {
    return `[${value.map((item) => stableStringify(item)).join(",")}]`;
  }
  const obj = value as Record<string, unknown>;
  const keys = Object.keys(obj).sort();
  return `{${keys.map((key) => `${JSON.stringify(key)}:${stableStringify(obj[key])}`).join(",")}}`;
}

function vertexSnapshot(vertex: PlanningVertex): PlanningMcSnapshotVertex {
  return {
    x: Number(vertex.x),
    y: Number(vertex.y),
  };
}

function presetForRadar(site: PlanningRadarSite): PlanningRadarPresetId | null {
  const preset = PLANNING_RADAR_PRESETS.find(
    (row) =>
      row.radar_type === site.radar_type &&
      row.detection_range_m === site.detection_range_m,
  );
  return preset?.id ?? null;
}

function radarSnapshot(site: PlanningRadarSite): PlanningMcSnapshotRadarSite {
  return {
    id: site.id,
    position: vertexSnapshot(site.position),
    radar_type: site.radar_type,
    radar_preset: presetForRadar(site),
    detection_range_m: Number(site.detection_range_m),
  };
}

function canonicalRadarGeometry(radars: PlanningRadarState): Array<{
  position: PlanningMcSnapshotVertex;
  radar_type: string;
  detection_range_m: number;
}> {
  return radars.sites
    .map((site) => ({
      position: vertexSnapshot(site.position),
      radar_type: site.radar_type,
      detection_range_m: Number(site.detection_range_m),
    }))
    .sort((a, b) => {
      if (a.position.x !== b.position.x) return a.position.x - b.position.x;
      if (a.position.y !== b.position.y) return a.position.y - b.position.y;
      if (a.detection_range_m !== b.detection_range_m) {
        return a.detection_range_m - b.detection_range_m;
      }
      return a.radar_type.localeCompare(b.radar_type);
    });
}

export function planningGeometryFingerprint(
  polygon: PlanningPolygonState,
  radars: PlanningRadarState,
): string {
  const canonical = {
    schema_version: PLANNING_MC_SNAPSHOT_SCHEMA_VERSION,
    polygon: {
      defense_area_vertices: (polygon.completedVertices ?? []).map(vertexSnapshot),
    },
    radars: {
      radar_sites: canonicalRadarGeometry(radars),
    },
  };
  return `rt_planning:sha256:${sha256Hex16(stableStringify(canonical))}`;
}

function analyticsSnapshot(
  analysis: PlanningCoverageAnalysis,
): PlanningMcSnapshotAnalyticsSummary {
  const recommendation = analysis.radarRecommendation;
  return {
    coverage_percent: analysis.estimate.coveragePercent,
    overlap_percent: analysis.overlapPercent,
    redundancy_percent: analysis.redundancyPercent,
    blind_spot_summary: analysis.blindSpotV2.summary,
    recommendation_summary: {
      suggested_radar: recommendation?.recommendedPresetLabel ?? null,
      suggested_position: recommendation
        ? vertexSnapshot(recommendation.approximatePlacement)
        : null,
      reason: recommendation?.reason ?? null,
    },
  };
}

export function buildPlanningMcSnapshot(
  polygon: PlanningPolygonState,
  radars: PlanningRadarState,
  analysis: PlanningCoverageAnalysis,
  options: BuildPlanningMcSnapshotOptions,
): PlanningMcSnapshotV1 {
  const createdUtc = options.createdUtc ?? utcNow();
  const extent = options.planningExtent ?? unifiedPlanningWorld();
  const planningGeometryId = planningGeometryFingerprint(polygon, radars);
  const snapshotCanonical = {
    schema_version: PLANNING_MC_SNAPSHOT_SCHEMA_VERSION,
    planning_geometry_id: planningGeometryId,
    planning_extent_id: extent.planning_extent_id,
    created_utc: createdUtc,
    source_layout_id: options.sourceLayoutId ?? null,
    source_geometry_id: options.sourceGeometryId ?? null,
  };
  return {
    schema_version: PLANNING_MC_SNAPSHOT_SCHEMA_VERSION,
    planning_snapshot_id: `rt_planning_snapshot:sha256:${sha256Hex16(stableStringify(snapshotCanonical))}`,
    planning_geometry_id: planningGeometryId,
    created_utc: createdUtc,
    polygon: {
      defense_area_vertices: (polygon.completedVertices ?? []).map(vertexSnapshot),
    },
    radars: {
      radar_sites: radars.sites.map(radarSnapshot),
    },
    analytics_summary: analyticsSnapshot(analysis),
    planning_extent: planningExtentMetadataForExport(extent),
    world_bounds_m: worldBoundsSnapshotMetadata(),
    presentation: {
      terrain_mode: options.terrainMode,
      selected_location_preset: options.selectedLocationPreset,
    },
    provenance: {
      ...(options.sourceLayoutId ? { source_layout_id: options.sourceLayoutId } : {}),
      ...(options.sourceGeometryId ? { source_geometry_id: options.sourceGeometryId } : {}),
      authority: "rt_planning_ui",
      origin: "platform/rt-sandbox-ui",
      ui_flow: "planning_mode",
    },
  };
}

export function exportPlanningMcSnapshotJson(snapshot: PlanningMcSnapshotV1): string {
  if (snapshot.schema_version !== PLANNING_MC_SNAPSHOT_SCHEMA_VERSION) {
    throw new Error(`expected schema_version ${PLANNING_MC_SNAPSHOT_SCHEMA_VERSION}`);
  }
  return exportJson(snapshot);
}

export function suggestedPlanningMcSnapshotFilename(
  snapshot: PlanningMcSnapshotV1,
): string {
  const safe = snapshot.planning_snapshot_id.replace(/[^\w.-]+/g, "_").slice(0, 64);
  return `${safe || "rt_planning_mc_snapshot"}.json`;
}

function triggerBrowserDownload(filename: string, json: string): void {
  const blob = new Blob([json], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

export function downloadPlanningMcSnapshot(snapshot: PlanningMcSnapshotV1): void {
  triggerBrowserDownload(
    suggestedPlanningMcSnapshotFilename(snapshot),
    exportPlanningMcSnapshotJson(snapshot),
  );
}

export async function copyPlanningMcSnapshot(
  snapshot: PlanningMcSnapshotV1,
): Promise<ClipboardResult> {
  return copyTextToClipboard(exportPlanningMcSnapshotJson(snapshot));
}
