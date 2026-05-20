import { Color, PolylineDashMaterialProperty, Viewer } from "cesium";
import type { ReplaySaBundle } from "@/replay/bundleSchema";

export type LosStatus = "visible" | "partially_occluded" | "terrain_blocked";

const STATUS_COLORS: Record<LosStatus, string> = {
  visible: "#4ade80",
  partially_occluded: "#fbbf24",
  terrain_blocked: "#f87171",
};

export function losStatusColor(status: string): Color {
  const hex = STATUS_COLORS[status as LosStatus] ?? "#94a3b8";
  return Color.fromCssColorString(hex).withAlpha(0.85);
}

export function filterLosSegmentsForT(
  bundle: ReplaySaBundle,
  currentT: number,
  options?: { maxSegments?: number; losScope?: "all" | "selected_track"; trackId?: string | null },
): NonNullable<ReplaySaBundle["los_segments"]> {
  const maxSegments = options?.maxSegments ?? 12;
  let segments = bundle.los_segments ?? [];
  if (options?.losScope === "selected_track" && options.trackId) {
    segments = segments.filter((s) => s.to_track_id === options.trackId);
  }
  const atT = segments.filter((s) => s.t === undefined || s.t === currentT);
  if (atT.length > 0) return atT.slice(0, maxSegments);
  return segments.slice(0, maxSegments);
}

export function isLosSegmentHighlighted(
  segment: { linked_event_ids?: string[] },
  selectedEventId: string | null,
  highlightedEventIds: string[],
): boolean {
  const linked = segment.linked_event_ids ?? [];
  if (!linked.length) return false;
  if (selectedEventId && linked.includes(selectedEventId)) return true;
  return highlightedEventIds.some((id) => linked.includes(id));
}

export function addLosSegmentEntities(
  viewer: Viewer,
  bundle: ReplaySaBundle,
  positions: (x: number, y: number, z?: number) => import("cesium").Cartesian3,
  currentT: number,
  selectedEventId: string | null,
  highlightedEventIds: string[],
  options?: { losScope?: "all" | "selected_track"; trackId?: string | null },
): void {
  const segments = filterLosSegmentsForT(bundle, currentT, {
    losScope: options?.losScope,
    trackId: options?.trackId,
  });
  for (const seg of segments) {
    const pts = seg.polyline_enu_m.map(([x, y, z]) => positions(x, y, z));
    if (pts.length < 2) continue;
    const highlight = isLosSegmentHighlighted(seg, selectedEventId, highlightedEventIds);
    const color = losStatusColor(seg.status);
    viewer.entities.add({
      id: seg.segment_id,
      polyline: {
        positions: pts,
        width: highlight ? 4 : 2.5,
        material:
          seg.status === "visible"
            ? color
            : new PolylineDashMaterialProperty({
                color,
                dashLength: seg.status === "terrain_blocked" ? 6 : 10,
              }),
      },
      description: `${seg.caveat}<br/><em>Explanatory LOS only.</em>`,
    });
  }
}

export const LOS_LEGEND = [
  { status: "visible", label: "Visible (replay proxy)" },
  { status: "partially_occluded", label: "Partially occluded" },
  { status: "terrain_blocked", label: "Terrain blocked" },
] as const;
