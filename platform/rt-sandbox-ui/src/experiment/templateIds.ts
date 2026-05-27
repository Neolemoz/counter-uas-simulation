/** Builtin RT runtime template ids (mirror template_catalog.py). */
export const BUILTIN_TEMPLATE_IDS = [
  "world_empty_v1",
  "radar_north_arc_v1",
  "radar_valley_pair_v1",
  "drone_ingress_lane_v1",
  "drone_patrol_pair_v1",
  "waypoint_patrol_triangle_v1",
  "interceptor_ready_pair_v1",
] as const;

export type BuiltinTemplateId = (typeof BUILTIN_TEMPLATE_IDS)[number];

export const BUILTIN_TEMPLATE_ID_SET = new Set<string>(BUILTIN_TEMPLATE_IDS);
