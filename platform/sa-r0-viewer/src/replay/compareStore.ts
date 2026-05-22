import { create } from "zustand";
import type { ReplaySaBundle } from "./bundleSchema";
import type { LayerVisibility, LosScope } from "./clockStore";
import { deriveReplayPresets } from "./clockStore";

export type CompareSlotId = "A" | "B";

export type ReplaySlotState = {
  bundle: ReplaySaBundle | null;
  currentT: number;
  playing: boolean;
  selectedEventId: string | null;
  highlightedTrackIds: string[];
  layers: LayerVisibility;
  losScope: LosScope;
  fitReplayNonce: number;
};

const DEFAULT_LAYERS: LayerVisibility = {
  sites: true,
  tracks: true,
  zones: true,
  overlays: true,
  losLinks: true,
  narrativeMarkers: true,
  spatial: {
    spatialAmbiguity: false,
    spatialLos: false,
    spatialSensitivity: false,
  },
};

function emptySlot(): ReplaySlotState {
  return {
    bundle: null,
    currentT: 0,
    playing: false,
    selectedEventId: null,
    highlightedTrackIds: [],
    layers: { ...DEFAULT_LAYERS },
    losScope: "all",
    fitReplayNonce: 0,
  };
}

function initSlot(bundle: ReplaySaBundle): ReplaySlotState {
  const start = bundle.clock.duration.start ?? 0;
  const presets = deriveReplayPresets(bundle);
  return {
    bundle,
    currentT: start,
    playing: false,
    selectedEventId: null,
    highlightedTrackIds: [],
    layers: { ...presets.layers },
    losScope: presets.losScope,
    fitReplayNonce: 0,
  };
}

type CompareState = {
  mode: "single" | "compare";
  slotA: ReplaySlotState;
  slotB: ReplaySlotState;
  syncClock: boolean;
  cameraLocked: boolean;
  emphasizeDelta: boolean;
  focusSlot: CompareSlotId;
  playbackMs: number;
  activePairId: string | null;
  enterCompare: (bundleA: ReplaySaBundle, bundleB: ReplaySaBundle, pairId?: string | null) => void;
  exitCompare: () => void;
  setSlotBundle: (slot: CompareSlotId, bundle: ReplaySaBundle | null) => void;
  setSlotCurrentT: (slot: CompareSlotId, t: number) => void;
  setSlotPlaying: (slot: CompareSlotId, playing: boolean) => void;
  setSlotSelectedEventId: (slot: CompareSlotId, id: string | null) => void;
  setSlotHighlightedTrackIds: (slot: CompareSlotId, ids: string[]) => void;
  toggleSlotLayer: (slot: CompareSlotId, key: keyof LayerVisibility) => void;
  setSlotLosScope: (slot: CompareSlotId, scope: LosScope) => void;
  requestSlotFitReplay: (slot: CompareSlotId) => void;
  setSyncClock: (v: boolean) => void;
  setCameraLocked: (v: boolean) => void;
  setEmphasizeDelta: (v: boolean) => void;
  setFocusSlot: (slot: CompareSlotId) => void;
  tickComparePlayback: () => void;
};

function clampT(bundle: ReplaySaBundle, t: number): number {
  const start = bundle.clock.duration.start ?? 0;
  const end = bundle.clock.duration.end ?? start;
  return Math.max(start, Math.min(end, t));
}

function updateSlot(
  slots: { slotA: ReplaySlotState; slotB: ReplaySlotState },
  slot: CompareSlotId,
  patch: Partial<ReplaySlotState>,
): { slotA: ReplaySlotState; slotB: ReplaySlotState } {
  const key = slot === "A" ? "slotA" : "slotB";
  return { ...slots, [key]: { ...slots[key], ...patch } };
}

export const useCompareStore = create<CompareState>((set, get) => ({
  mode: "single",
  slotA: emptySlot(),
  slotB: emptySlot(),
  syncClock: false,
  cameraLocked: true,
  emphasizeDelta: true,
  focusSlot: "A",
  playbackMs: 400,
  activePairId: null,
  enterCompare: (bundleA, bundleB, pairId = null) => {
    set({
      mode: "compare",
      slotA: initSlot(bundleA),
      slotB: initSlot(bundleB),
      syncClock: false,
      cameraLocked: true,
      emphasizeDelta: true,
      focusSlot: "A",
      activePairId: pairId,
    });
  },
  exitCompare: () =>
    set({
      mode: "single",
      slotA: emptySlot(),
      slotB: emptySlot(),
      activePairId: null,
    }),
  setSlotBundle: (slot, bundle) => {
    if (!bundle) {
      set((s) => updateSlot(s, slot, emptySlot()));
      return;
    }
    set((s) => updateSlot(s, slot, initSlot(bundle)));
  },
  setSlotCurrentT: (slot, t) => {
    const { syncClock, slotA, slotB } = get();
    const primary = slot === "A" ? slotA : slotB;
    if (!primary.bundle) return;
    const clamped = clampT(primary.bundle, t);
    if (syncClock) {
      const otherSlot: CompareSlotId = slot === "A" ? "B" : "A";
      const other = otherSlot === "A" ? slotA : slotB;
      const otherT = other.bundle ? clampT(other.bundle, clamped) : clamped;
      set((s) => {
        let next = updateSlot(s, slot, { currentT: clamped });
        next = updateSlot(next, otherSlot, { currentT: otherT });
        return next;
      });
    } else {
      set((s) => updateSlot(s, slot, { currentT: clamped }));
    }
  },
  setSlotPlaying: (slot, playing) => {
    const { syncClock } = get();
    if (syncClock) {
      set((s) => ({
        slotA: { ...s.slotA, playing },
        slotB: { ...s.slotB, playing },
      }));
    } else {
      set((s) => updateSlot(s, slot, { playing }));
    }
  },
  setSlotSelectedEventId: (slot, id) => set((s) => updateSlot(s, slot, { selectedEventId: id })),
  setSlotHighlightedTrackIds: (slot, ids) =>
    set((s) => updateSlot(s, slot, { highlightedTrackIds: ids })),
  toggleSlotLayer: (slot, key) =>
    set((s) => {
      const cur = slot === "A" ? s.slotA : s.slotB;
      if (key === "spatial") return s;
      return updateSlot(s, slot, {
        layers: { ...cur.layers, [key]: !cur.layers[key] },
      });
    }),
  setSlotLosScope: (slot, scope) => set((s) => updateSlot(s, slot, { losScope: scope })),
  requestSlotFitReplay: (slot) =>
    set((s) => {
      const cur = slot === "A" ? s.slotA : s.slotB;
      return updateSlot(s, slot, { fitReplayNonce: cur.fitReplayNonce + 1 });
    }),
  setSyncClock: (v) => set({ syncClock: v }),
  setCameraLocked: (v) => set({ cameraLocked: v }),
  setEmphasizeDelta: (v) => set({ emphasizeDelta: v }),
  setFocusSlot: (slot) => set({ focusSlot: slot }),
  tickComparePlayback: () => {
    const { slotA, slotB, syncClock } = get();
    const leader = syncClock ? slotA : slotA.playing ? slotA : slotB;
    if (!leader.bundle || !leader.playing) return;
    const step = leader.bundle.clock.duration.step ?? 1;
    const end = leader.bundle.clock.duration.end ?? 0;
    const start = leader.bundle.clock.duration.start ?? 0;
    const next = leader.currentT + step;
    if (next > end) {
      set({
        slotA: { ...slotA, currentT: start, playing: false },
        slotB: { ...slotB, currentT: slotB.bundle ? clampT(slotB.bundle, start) : 0, playing: false },
      });
      return;
    }
    get().setSlotCurrentT("A", next);
    if (syncClock) get().setSlotCurrentT("B", next);
  },
}));

export function getCompareSlot(slot: CompareSlotId): ReplaySlotState {
  const s = useCompareStore.getState();
  return slot === "A" ? s.slotA : s.slotB;
}
