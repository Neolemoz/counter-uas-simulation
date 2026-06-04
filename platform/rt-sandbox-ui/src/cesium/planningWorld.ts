import { WORLD_AXIS_HALF_EXTENT_M } from "@/world/bounds";

export type LegacyPlanningExtentId = "planning_5km" | "planning_10km" | "planning_20km";

export type PlanningExtentId = "planning_unified_7km" | LegacyPlanningExtentId;

export interface PlanningExtent {
  planning_extent_id: PlanningExtentId;
  planning_extent_radius_m: number;
  planning_extent_label: string;
}

export const UNIFIED_PLANNING_WORLD: PlanningExtent = {
  planning_extent_id: "planning_unified_7km",
  planning_extent_radius_m: WORLD_AXIS_HALF_EXTENT_M,
  planning_extent_label: "Unified 7 km World",
};

/** @deprecated Legacy snapshot metadata only — not world authority. */
export const LEGACY_PLANNING_EXTENTS: PlanningExtent[] = [
  {
    planning_extent_id: "planning_5km",
    planning_extent_radius_m: 5_000,
    planning_extent_label: "5 km Planning World (legacy)",
  },
  {
    planning_extent_id: "planning_10km",
    planning_extent_radius_m: 10_000,
    planning_extent_label: "10 km Planning World (legacy)",
  },
  {
    planning_extent_id: "planning_20km",
    planning_extent_radius_m: 20_000,
    planning_extent_label: "20 km Planning World (legacy)",
  },
];

export const LEGACY_PLANNING_EXTENT_IDS: LegacyPlanningExtentId[] = [
  "planning_5km",
  "planning_10km",
  "planning_20km",
];

export const DEFAULT_PLANNING_EXTENT_ID: PlanningExtentId = "planning_unified_7km";

export const PLANNING_EXTENT_GOVERNANCE_COPY =
  "Planning operates on the unified 7 km world (±7000 m). Legacy planning extent IDs remain readable in imported snapshots only; they are not world authority.";

export function unifiedPlanningWorld(): PlanningExtent {
  return UNIFIED_PLANNING_WORLD;
}

export function isLegacyPlanningExtentId(id: string): id is LegacyPlanningExtentId {
  return (LEGACY_PLANNING_EXTENT_IDS as readonly string[]).includes(id);
}

export function planningExtentById(id: PlanningExtentId | string): PlanningExtent {
  if (id === UNIFIED_PLANNING_WORLD.planning_extent_id) {
    return UNIFIED_PLANNING_WORLD;
  }
  const legacy = LEGACY_PLANNING_EXTENTS.find((extent) => extent.planning_extent_id === id);
  if (legacy) return legacy;
  return UNIFIED_PLANNING_WORLD;
}

export function planningExtentMetadata(extent: PlanningExtent): PlanningExtent {
  return {
    planning_extent_id: extent.planning_extent_id,
    planning_extent_radius_m: extent.planning_extent_radius_m,
    planning_extent_label: extent.planning_extent_label,
  };
}

export function planningExtentMetadataForExport(
  extent?: PlanningExtent,
): PlanningExtent {
  if (extent && isLegacyPlanningExtentId(extent.planning_extent_id)) {
    return planningExtentMetadata(extent);
  }
  return planningExtentMetadata(UNIFIED_PLANNING_WORLD);
}
