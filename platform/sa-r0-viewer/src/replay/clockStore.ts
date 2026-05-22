import { create } from "zustand";
import type { ReplaySaBundle } from "./bundleSchema";

export type SpatialLayerVisibility = {
  spatialAmbiguity: boolean;
  spatialLos: boolean;
  spatialSensitivity: boolean;
};

export type LayerVisibility = {
  sites: boolean;
  tracks: boolean;
  zones: boolean;
  overlays: boolean;
  losLinks: boolean;
  narrativeMarkers: boolean;
  spatial: SpatialLayerVisibility;
};

export type LosScope = "all" | "selected_track";

const DEFAULT_SPATIAL: SpatialLayerVisibility = {
  spatialAmbiguity: false,
  spatialLos: false,
  spatialSensitivity: false,
};

const DEFAULT_LAYERS: LayerVisibility = {
  sites: true,
  tracks: true,
  zones: true,
  overlays: true,
  losLinks: true,
  narrativeMarkers: true,
  spatial: { ...DEFAULT_SPATIAL },
};

export function deriveReplayPresets(bundle: ReplaySaBundle): { layers: LayerVisibility; losScope: LosScope } {
  const tags = bundle.scenario.replay_tags ?? [];
  const amb = bundle.scenario.ambiguity_profile?.level;
  const multiTrack = bundle.tracks.length > 2;
  const layers = { ...DEFAULT_LAYERS };
  let losScope: LosScope = "all";

  if (
    tags.includes("multi_threat") ||
    tags.includes("assignment_ambiguity") ||
    amb === "high" ||
    amb === "saturation" ||
    multiTrack
  ) {
    losScope = "selected_track";
  }
  if (tags.includes("urban_clutter") || tags.includes("cluttered_replay")) {
    layers.overlays = true;
  }

  return { layers, losScope };
}

type ClockState = {
  bundle: ReplaySaBundle | null;
  currentT: number;
  playing: boolean;
  playbackMs: number;
  selectedEventId: string | null;
  highlightedTrackIds: string[];
  layers: LayerVisibility;
  losScope: LosScope;
  fitReplayNonce: number;
  setBundle: (bundle: ReplaySaBundle | null) => void;
  applyBundlePresets: (bundle: ReplaySaBundle) => void;
  setCurrentT: (t: number) => void;
  setPlaying: (playing: boolean) => void;
  setSelectedEventId: (id: string | null) => void;
  setHighlightedTrackIds: (ids: string[]) => void;
  toggleLayer: (key: keyof LayerVisibility) => void;
  toggleSpatialLayer: (key: keyof SpatialLayerVisibility) => void;
  setLosScope: (scope: LosScope) => void;
  tickPlayback: () => void;
  requestFitReplay: () => void;
};

export const useClockStore = create<ClockState>((set, get) => ({
  bundle: null,
  currentT: 0,
  playing: false,
  playbackMs: 400,
  selectedEventId: null,
  highlightedTrackIds: [],
  fitReplayNonce: 0,
  layers: DEFAULT_LAYERS,
  losScope: "all",
  setBundle: (bundle) => {
    if (!bundle) {
      set({
        bundle: null,
        currentT: 0,
        playing: false,
        selectedEventId: null,
        layers: DEFAULT_LAYERS,
        losScope: "all",
      });
      return;
    }
    const start = bundle.clock.duration.start ?? 0;
    const presets = deriveReplayPresets(bundle);
    set({
      bundle,
      currentT: start,
      playing: false,
      selectedEventId: null,
      highlightedTrackIds: [],
      ...presets,
    });
  },
  applyBundlePresets: (bundle) => {
    set(deriveReplayPresets(bundle));
  },
  setCurrentT: (t) => set({ currentT: t }),
  setPlaying: (playing) => set({ playing }),
  setSelectedEventId: (id) => set({ selectedEventId: id }),
  setHighlightedTrackIds: (ids) => set({ highlightedTrackIds: ids }),
  toggleLayer: (key) =>
    set((s) => {
      if (key === "spatial") return s;
      return { layers: { ...s.layers, [key]: !s.layers[key] } };
    }),
  toggleSpatialLayer: (key) =>
    set((s) => ({
      layers: {
        ...s.layers,
        spatial: { ...s.layers.spatial, [key]: !s.layers.spatial[key] },
      },
    })),
  setLosScope: (scope) => set({ losScope: scope }),
  requestFitReplay: () =>
    set((s) => ({ fitReplayNonce: s.fitReplayNonce + 1 })),
  tickPlayback: () => {
    const { bundle, currentT, playing } = get();
    if (!bundle || !playing) return;
    const { end, step } = bundle.clock.duration;
    const next = currentT + step;
    if (next > end) {
      set({ currentT: bundle.clock.duration.start, playing: false });
    } else {
      set({ currentT: next });
    }
  },
}));

export function primaryThreatTrackId(bundle: ReplaySaBundle): string | null {
  const threat = bundle.tracks.find((t) => t.role === "threat");
  return threat?.track_id ?? bundle.tracks[0]?.track_id ?? null;
}
