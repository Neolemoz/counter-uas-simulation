import { Color } from "cesium";

export const ZONE_DISPLAY_LABELS: Record<string, string> = {
  outer_detection: "Outer Detection",
  tracking: "Tracking",
  engagement: "Engagement",
  protected: "Critical / Protected Area",
};

export const ZONE_SCENARIO_CAVEAT =
  "Scenario policy overlay — not validated doctrine or operational boundary.";

const ZONE_COLORS: Record<string, string> = {
  outer_detection: "#3b82f6",
  tracking: "#8b5cf6",
  engagement: "#f59e0b",
  protected: "#ef4444",
};

export function zoneDisplayLabel(zoneId: string, bundleLabel?: string): string {
  return bundleLabel ?? ZONE_DISPLAY_LABELS[zoneId] ?? zoneId;
}

export function zoneFillColor(zoneId: string, overlaysVisible = false): Color {
  const hex = ZONE_COLORS[zoneId] ?? "#64748b";
  const alpha = overlaysVisible ? 0.035 : 0.06;
  return Color.fromCssColorString(hex).withAlpha(alpha);
}

export function zoneOutlineColor(zoneId: string): Color {
  const hex = ZONE_COLORS[zoneId] ?? "#64748b";
  return Color.fromCssColorString(hex).withAlpha(0.4);
}

export function sortZonesByRadiusDesc<T extends { geometry: { radius_m?: number } }>(zones: T[]): T[] {
  return [...zones].sort(
    (a, b) => (b.geometry.radius_m ?? 0) - (a.geometry.radius_m ?? 0),
  );
}
