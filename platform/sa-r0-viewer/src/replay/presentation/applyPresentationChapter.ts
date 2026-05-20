import type { SpatialDeclutterMode } from "../useSweepStore";
import type { LayerVisibility, LosScope, SpatialLayerVisibility } from "../clockStore";
import type { ReplaySaBundle } from "../bundleSchema";
import type { PresentationChapter } from "./presentationStore";

const EMPHASIS_LAYERS: Record<string, Partial<LayerVisibility>> = {
  ambiguity: { spatial: { spatialAmbiguity: true, spatialLos: false, spatialSensitivity: false } },
  los: { spatial: { spatialAmbiguity: false, spatialLos: true, spatialSensitivity: false }, losLinks: true },
  topology: { overlays: true, zones: true },
  assignment: { narrativeMarkers: true, tracks: true },
  pacing: { narrativeMarkers: true },
};

export function applyChapterToLayers(
  base: LayerVisibility,
  chapter: PresentationChapter | null,
): LayerVisibility {
  if (!chapter) return base;
  const emphasis = chapter.narrative_emphasis;
  const merged = { ...base, spatial: { ...base.spatial } };
  if (emphasis && EMPHASIS_LAYERS[emphasis]) {
    const patch = EMPHASIS_LAYERS[emphasis]!;
    if (patch.spatial) {
      merged.spatial = { ...merged.spatial, ...patch.spatial };
    }
    if (patch.overlays != null) merged.overlays = patch.overlays;
    if (patch.zones != null) merged.zones = patch.zones;
    if (patch.losLinks != null) merged.losLinks = patch.losLinks;
    if (patch.narrativeMarkers != null) merged.narrativeMarkers = patch.narrativeMarkers;
    if (patch.tracks != null) merged.tracks = patch.tracks;
  }
  if (chapter.visible_layers) {
    for (const [k, v] of Object.entries(chapter.visible_layers)) {
      if (k === "spatial") continue;
      if (k in merged && typeof v === "boolean") {
        (merged as Record<string, boolean | SpatialLayerVisibility>)[k] = v;
      }
    }
  }
  return merged;
}

export function chapterLosScope(chapter: PresentationChapter | null, fallback: LosScope): LosScope {
  return chapter?.los_scope ?? fallback;
}

export function chapterSpatialDeclutter(chapter: PresentationChapter | null): SpatialDeclutterMode {
  return chapter?.spatial_declutter ?? "top_k";
}

export function applyChapterClock(
  bundle: ReplaySaBundle,
  chapter: PresentationChapter,
): { t: number; eventId: string | null } {
  const focus = chapter.focus_event_ids?.[0];
  if (focus) {
    const ev = bundle.narrative.events.find((e) => e.event_id === focus);
    if (ev && typeof ev.line_index === "number") {
      return { t: ev.line_index, eventId: focus };
    }
  }
  return { t: chapter.t_start, eventId: focus ?? null };
}

export function timelineRange(
  bundle: ReplaySaBundle,
  chapter: PresentationChapter | null,
  compressed: boolean,
): { start: number; end: number } {
  const { start, end } = bundle.clock.duration;
  if (!compressed || !chapter) return { start, end };
  return { start: chapter.t_start, end: chapter.t_end };
}
