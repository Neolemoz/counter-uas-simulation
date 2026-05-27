/** Frozen RT-S3 entity catalog. */

export type EntityType = "radar" | "interceptor" | "drone" | "waypoint_marker";

export const ENTITY_TYPES: EntityType[] = [
  "radar",
  "interceptor",
  "drone",
  "waypoint_marker",
];

export const ENTITY_GLYPHS: Record<EntityType, string> = {
  radar: "R",
  interceptor: "I",
  drone: "D",
  waypoint_marker: "W",
};

export const ENTITY_LABELS: Record<EntityType, string> = {
  radar: "Radar",
  interceptor: "Interceptor",
  drone: "Drone",
  waypoint_marker: "Waypoint",
};

export const DEFAULT_Z: Record<EntityType, number> = {
  radar: 10,
  interceptor: 10,
  drone: 10,
  waypoint_marker: 5,
};

export function isEntityType(value: string): value is EntityType {
  return (ENTITY_TYPES as string[]).includes(value);
}

export function defaultPose(entityType: EntityType, x: number, y: number) {
  return {
    x,
    y,
    z: DEFAULT_Z[entityType],
    yaw_deg: 0,
  };
}
