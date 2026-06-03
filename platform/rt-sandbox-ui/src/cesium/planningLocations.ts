export type PlanningLocationPresetId = "bangkok" | "chiang_mai" | "phuket" | "custom";

export interface PlanningLocationPreset {
  id: PlanningLocationPresetId;
  label: string;
  latitudeDeg: number;
  longitudeDeg: number;
}

export const PLANNING_LOCATION_PRESETS: PlanningLocationPreset[] = [
  { id: "bangkok", label: "Bangkok", latitudeDeg: 13.7563, longitudeDeg: 100.5018 },
  { id: "chiang_mai", label: "Chiang Mai", latitudeDeg: 18.7883, longitudeDeg: 98.9853 },
  { id: "phuket", label: "Phuket", latitudeDeg: 7.8804, longitudeDeg: 98.3923 },
  { id: "custom", label: "Custom Coordinates", latitudeDeg: 13.7563, longitudeDeg: 100.5018 },
];

export const DEFAULT_PLANNING_LOCATION_PRESET_ID: PlanningLocationPresetId = "bangkok";

export const PLANNING_LOCATION_GOVERNANCE_COPY =
  "Real-world locations are presentation-only; they do not affect runtime simulation, sensors, LOS, or MC.";

export function planningLocationPreset(
  id: PlanningLocationPresetId,
): PlanningLocationPreset {
  return (
    PLANNING_LOCATION_PRESETS.find((preset) => preset.id === id) ??
    PLANNING_LOCATION_PRESETS[0]
  );
}

export function validatePlanningLatitude(value: string): number | null {
  if (value.trim() === "") return null;
  const parsed = Number(value);
  if (!Number.isFinite(parsed) || parsed < -90 || parsed > 90) return null;
  return parsed;
}

export function validatePlanningLongitude(value: string): number | null {
  if (value.trim() === "") return null;
  const parsed = Number(value);
  if (!Number.isFinite(parsed) || parsed < -180 || parsed > 180) return null;
  return parsed;
}

export function validatedPlanningCoordinates(
  latitude: string,
  longitude: string,
): { latitudeDeg: number; longitudeDeg: number } | null {
  const latitudeDeg = validatePlanningLatitude(latitude);
  const longitudeDeg = validatePlanningLongitude(longitude);
  if (latitudeDeg === null || longitudeDeg === null) return null;
  return { latitudeDeg, longitudeDeg };
}
