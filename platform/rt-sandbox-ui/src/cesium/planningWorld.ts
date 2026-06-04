export type PlanningExtentId = "planning_5km" | "planning_10km" | "planning_20km";

export interface PlanningExtent {
  planning_extent_id: PlanningExtentId;
  planning_extent_radius_m: number;
  planning_extent_label: string;
}

export const PLANNING_EXTENTS: PlanningExtent[] = [
  {
    planning_extent_id: "planning_5km",
    planning_extent_radius_m: 5_000,
    planning_extent_label: "5 km Planning World",
  },
  {
    planning_extent_id: "planning_10km",
    planning_extent_radius_m: 10_000,
    planning_extent_label: "10 km Planning World",
  },
  {
    planning_extent_id: "planning_20km",
    planning_extent_radius_m: 20_000,
    planning_extent_label: "20 km Planning World",
  },
];

export const DEFAULT_PLANNING_EXTENT_ID: PlanningExtentId = "planning_10km";

export const PLANNING_EXTENT_GOVERNANCE_COPY =
  "Planning extent is UI-local metadata only; it is not runtime authority, bridge bounds, or MC execution authority.";

export function planningExtentById(id: PlanningExtentId): PlanningExtent {
  return (
    PLANNING_EXTENTS.find((extent) => extent.planning_extent_id === id) ??
    PLANNING_EXTENTS[1]
  );
}

export function planningExtentMetadata(extent: PlanningExtent): PlanningExtent {
  return {
    planning_extent_id: extent.planning_extent_id,
    planning_extent_radius_m: extent.planning_extent_radius_m,
    planning_extent_label: extent.planning_extent_label,
  };
}

