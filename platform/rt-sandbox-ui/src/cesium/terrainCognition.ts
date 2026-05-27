import type { MirrorEntity } from "./entityMarkers";
import type { TerrainLayerVisibility } from "./terrainLayers";
import { activeTerrainLayerLabels } from "./terrainLayers";
import {
  contourLevelUnderM,
  displayAglM,
  listOcclusionMarkers,
  NOMINAL_SENSOR_DOME_RADIUS_M,
  nearestRidgeLabel,
  sampleTerrainHeight,
} from "./rtFictionalTerrain";
import { countEntitiesInNominalDome } from "./sensorDomeLayer";

export type OcclusionStatus = "clear" | "terrain_blocked" | "marker_occluded";

export interface EntityTerrainRelation {
  terrain_m: number;
  registry_z_m: number;
  display_agl_m: number;
  sim_agl_m?: number | null;
  nearest_ridge: string | null;
  contour_level_m: number | null;
}

export function entityTerrainRelation(
  x_m: number,
  y_m: number,
  registry_z_m: number,
  sim_agl_m?: number | null,
): EntityTerrainRelation {
  const terrain_m = sampleTerrainHeight(x_m, y_m);
  const relation: EntityTerrainRelation = {
    terrain_m,
    registry_z_m,
    display_agl_m: displayAglM(x_m, y_m, registry_z_m),
    nearest_ridge: nearestRidgeLabel(x_m, y_m),
    contour_level_m: contourLevelUnderM(x_m, y_m),
  };
  if (typeof sim_agl_m === "number" && Number.isFinite(sim_agl_m)) {
    relation.sim_agl_m = sim_agl_m;
  }
  return relation;
}

export function terrainContextLine(relation: EntityTerrainRelation): string {
  const crest =
    relation.contour_level_m != null
      ? `contour band ≥${relation.contour_level_m}m (heuristic)`
      : "below lowest contour band";
  return `${crest}; nearest ridge: ${relation.nearest_ridge ?? "—"}`;
}

function terrainBlocksSegment(
  ax: number,
  ay: number,
  az: number,
  bx: number,
  by: number,
  bz: number,
  steps = 12,
): boolean {
  for (let i = 1; i < steps; i++) {
    const t = i / steps;
    const x = ax + (bx - ax) * t;
    const y = ay + (by - ay) * t;
    const lineZ = az + (bz - az) * t;
    const terrain = sampleTerrainHeight(x, y);
    if (terrain > lineZ + 2) return true;
  }
  return false;
}

function markerBlocksSegment(ax: number, ay: number, bx: number, by: number): boolean {
  for (const m of listOcclusionMarkers()) {
    const [mx, my] = m.position_enu_m;
    const d = distancePointToSegment2D(mx, my, ax, ay, bx, by);
    if (d < 35) return true;
  }
  return false;
}

function distancePointToSegment2D(
  px: number,
  py: number,
  ax: number,
  ay: number,
  bx: number,
  by: number,
): number {
  const dx = bx - ax;
  const dy = by - ay;
  const len2 = dx * dx + dy * dy;
  if (len2 < 1e-6) {
    return Math.hypot(px - ax, py - ay);
  }
  let t = ((px - ax) * dx + (py - ay) * dy) / len2;
  t = Math.max(0, Math.min(1, t));
  const qx = ax + t * dx;
  const qy = ay + t * dy;
  return Math.hypot(px - qx, py - qy);
}

export function occlusionHeuristic(
  from: MirrorEntity,
  to: MirrorEntity,
): OcclusionStatus {
  const ax = Number(from.pose.x ?? 0);
  const ay = Number(from.pose.y ?? 0);
  const az = Number(from.pose.z ?? 0);
  const bx = Number(to.pose.x ?? 0);
  const by = Number(to.pose.y ?? 0);
  const bz = Number(to.pose.z ?? 0);
  if (markerBlocksSegment(ax, ay, bx, by)) return "marker_occluded";
  if (terrainBlocksSegment(ax, ay, az, bx, by, bz)) return "terrain_blocked";
  return "clear";
}

export function losCueSummary(from: MirrorEntity, to: MirrorEntity): string {
  const status = occlusionHeuristic(from, to);
  const shortTo = `${to.entity_type} · ${to.entity_id.slice(0, 8)}`;
  return `LOS heuristic: ${status} → ${shortTo}`;
}

export function visibilityHint(
  selected: MirrorEntity,
  entities: MirrorEntity[],
): string {
  const occ = nearestOcclusionTarget(selected, entities);
  if (!occ) return "No peer entity for visibility heuristic";
  if (occ.status === "terrain_blocked") {
    return "Ridge-masked heuristic — fictional terrain may block line of sight";
  }
  if (occ.status === "marker_occluded") {
    return "Landmark-occluded heuristic — explanatory marker near segment";
  }
  return "Open-sky heuristic — segment clear on fictional heightmap";
}

export function nearestOcclusionTarget(
  selected: MirrorEntity,
  entities: MirrorEntity[],
): { target: MirrorEntity; status: OcclusionStatus } | null {
  let best: { target: MirrorEntity; status: OcclusionStatus; dist: number } | null = null;
  for (const ent of entities) {
    if (ent.entity_id === selected.entity_id) continue;
    const dx = Number(ent.pose.x ?? 0) - Number(selected.pose.x ?? 0);
    const dy = Number(ent.pose.y ?? 0) - Number(selected.pose.y ?? 0);
    const dist = Math.hypot(dx, dy);
    if (!best || dist < best.dist) {
      best = {
        target: ent,
        status: occlusionHeuristic(selected, ent),
        dist,
      };
    }
  }
  return best ? { target: best.target, status: best.status } : null;
}

export interface SensorDomeContext {
  radius_m: number;
  entities_in_dome: number;
  note: string;
  context_label: string;
}

export function sensorDomeContext(
  radar: MirrorEntity,
  entities: MirrorEntity[],
): SensorDomeContext {
  const x = Number(radar.pose.x ?? 0);
  const y = Number(radar.pose.y ?? 0);
  const count = countEntitiesInNominalDome(x, y, entities);
  return {
    radius_m: NOMINAL_SENSOR_DOME_RADIUS_M,
    entities_in_dome: count,
    note: "Nominal dome — not sensor coverage proof",
    context_label: `${count} entities within ${NOMINAL_SENSOR_DOME_RADIUS_M}m horizontal (explanatory)`,
  };
}

export function terrainHubSummary(
  layersEnabled: boolean,
  layers?: TerrainLayerVisibility,
): string | null {
  if (!layersEnabled) return null;
  const active = layers ? activeTerrainLayerLabels(layers).join(", ") : "terrain";
  return `Terrain layers on (${active}) — fictional ENU heightmap; LOS and visibility cues are heuristic only`;
}
