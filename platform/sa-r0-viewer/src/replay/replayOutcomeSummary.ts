import type { ReplaySaBundle } from "./bundleSchema";

export type ReplayOutcomeSummary = {
  durationSpan: number;
  durationStart: number;
  durationEnd: number;
  firstDetectionT: number | null;
  firstSelectionT: number | null;
  firstAmbiguityT: number | null;
  ambiguityWindowCount: number;
  losDegradedCount: number;
  losVisibleCount: number;
  eventCountByCategory: Record<string, number>;
};

export function extractReplayOutcome(bundle: ReplaySaBundle): ReplayOutcomeSummary {
  const start = bundle.clock.duration.start ?? 0;
  const end = bundle.clock.duration.end ?? 0;
  const markers = bundle.clock.markers ?? [];
  const detectionMarker = markers.find((m) => m.category === "detection");
  const selectionMarker = markers.find((m) => m.category === "selection");
  const events = bundle.narrative.events ?? [];

  const firstDetectionEvent = events.find((e) => e.category === "detection");
  const firstSelectionEvent = events.find((e) => e.category === "selection");
  const firstAmbiguityEvent = events.find((e) => e.category === "ambiguity");

  const eventCountByCategory: Record<string, number> = {};
  for (const ev of events) {
    const cat = String(ev.category ?? "other");
    eventCountByCategory[cat] = (eventCountByCategory[cat] ?? 0) + 1;
  }

  const los = bundle.los_segments ?? [];
  const losDegradedCount = los.filter(
    (s) => s.status === "partially_occluded" || s.status === "terrain_blocked",
  ).length;
  const losVisibleCount = los.filter((s) => s.status === "visible").length;

  return {
    durationSpan: end - start + 1,
    durationStart: start,
    durationEnd: end,
    firstDetectionT:
      (detectionMarker?.t as number | undefined) ??
      (firstDetectionEvent?.line_index as number | undefined) ??
      null,
    firstSelectionT:
      (selectionMarker?.t as number | undefined) ??
      (firstSelectionEvent?.line_index as number | undefined) ??
      null,
    firstAmbiguityT: (firstAmbiguityEvent?.line_index as number | undefined) ?? null,
    ambiguityWindowCount: (bundle.narrative.windows ?? []).length,
    losDegradedCount,
    losVisibleCount,
    eventCountByCategory,
  };
}

export function formatDeltaT(a: number | null, b: number | null): string {
  if (a == null || b == null) return "—";
  const d = b - a;
  if (d === 0) return "0";
  return d > 0 ? `+${d}` : `${d}`;
}
