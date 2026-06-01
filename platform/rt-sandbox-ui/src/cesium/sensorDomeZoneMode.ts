/** UI-only sensor dome visibility: radar detection vs protected defense zones. */

export type SensorDomeZoneMode = "both" | "radar" | "defense";

export const DEFAULT_SENSOR_DOME_ZONE_MODE: SensorDomeZoneMode = "both";

export function shouldShowRadarZones(mode: SensorDomeZoneMode = "both"): boolean {
  return mode === "both" || mode === "radar";
}

export function shouldShowDefenseZones(mode: SensorDomeZoneMode = "both"): boolean {
  return mode === "both" || mode === "defense";
}
