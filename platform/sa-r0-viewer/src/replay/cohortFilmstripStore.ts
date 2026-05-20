import { create } from "zustand";
import type { ReplaySaBundle } from "./bundleSchema";
import type { LayerVisibility, LosScope } from "./clockStore";
import { deriveReplayPresets } from "./clockStore";
import { loadBundleFromUrl } from "./loadBundle";
import type { ReplayMcSweep } from "./sweepSchema";

export const MAX_FILMSTRIP_SLOTS = 4;

export type FilmstripSlotState = {
  memberIndex: number;
  bundle: ReplaySaBundle | null;
  currentT: number;
  playing: boolean;
  selectedEventId: string | null;
  highlightedTrackIds: string[];
  layers: LayerVisibility;
  losScope: LosScope;
  fitReplayNonce: number;
};

function initSlot(bundle: ReplaySaBundle, memberIndex: number): FilmstripSlotState {
  const start = bundle.clock.duration.start ?? 0;
  const presets = deriveReplayPresets(bundle);
  return {
    memberIndex,
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

type FilmstripState = {
  mode: "off" | "filmstrip";
  sweep: ReplayMcSweep | null;
  slots: FilmstripSlotState[];
  syncClock: boolean;
  focusSlotIndex: number;
  playbackMs: number;
  enterFilmstrip: (sweep: ReplayMcSweep, memberIndices: number[]) => Promise<void>;
  exitFilmstrip: () => void;
  setSlotCurrentT: (slotIndex: number, t: number) => void;
  setSlotPlaying: (slotIndex: number, playing: boolean) => void;
  setSyncClock: (v: boolean) => void;
  setFocusSlotIndex: (i: number) => void;
  tickFilmstripPlayback: () => void;
};

function clampT(bundle: ReplaySaBundle, t: number): number {
  const start = bundle.clock.duration.start ?? 0;
  const end = bundle.clock.duration.end ?? start;
  return Math.max(start, Math.min(end, t));
}

export const useCohortFilmstripStore = create<FilmstripState>((set, get) => ({
  mode: "off",
  sweep: null,
  slots: [],
  syncClock: true,
  focusSlotIndex: 0,
  playbackMs: 400,
  enterFilmstrip: async (sweep, memberIndices) => {
    const indices = memberIndices.slice(0, MAX_FILMSTRIP_SLOTS);
    if (indices.length < 2) return;
    const slots: FilmstripSlotState[] = [];
    for (const idx of indices) {
      const member = sweep.members[idx];
      if (!member) continue;
      const bundle = await loadBundleFromUrl(member.demo_bundle_url);
      slots.push(initSlot(bundle, idx));
    }
    set({
      mode: "filmstrip",
      sweep,
      slots,
      syncClock: true,
      focusSlotIndex: 0,
    });
    const url = new URL(window.location.href);
    url.searchParams.set("sweep", sweep.sweep_id);
    url.searchParams.set("filmstrip", indices.join(","));
    url.searchParams.delete("cohort");
    url.searchParams.delete("member");
    window.history.replaceState({}, "", url.toString());
  },
  exitFilmstrip: () => {
    set({ mode: "off", sweep: null, slots: [] });
    const url = new URL(window.location.href);
    url.searchParams.delete("filmstrip");
    window.history.replaceState({}, "", url.toString());
  },
  setSlotCurrentT: (slotIndex, t) => {
    const { syncClock, slots } = get();
    const slot = slots[slotIndex];
    if (!slot?.bundle) return;
    const clamped = clampT(slot.bundle, t);
    if (syncClock) {
      set({
        slots: slots.map((s) =>
          s.bundle ? { ...s, currentT: clampT(s.bundle, clamped) } : s,
        ),
      });
    } else {
      set({
        slots: slots.map((s, i) => (i === slotIndex ? { ...s, currentT: clamped } : s)),
      });
    }
  },
  setSlotPlaying: (slotIndex, playing) => {
    const { syncClock, slots } = get();
    if (syncClock) {
      set({ slots: slots.map((s) => ({ ...s, playing })) });
    } else {
      set({
        slots: slots.map((s, i) => (i === slotIndex ? { ...s, playing } : s)),
      });
    }
  },
  setSyncClock: (syncClock) => set({ syncClock }),
  setFocusSlotIndex: (focusSlotIndex) => set({ focusSlotIndex }),
  tickFilmstripPlayback: () => {
    const { slots, syncClock } = get();
    const leader = slots.find((s) => s.playing && s.bundle) ?? slots[0];
    if (!leader?.bundle || !leader.playing) return;
    const step = leader.bundle.clock.duration.step ?? 1;
    const end = leader.bundle.clock.duration.end ?? 0;
    const next = leader.currentT + step;
    if (next > end) {
      set({
        slots: slots.map((s) => ({
          ...s,
          currentT: s.bundle ? (s.bundle.clock.duration.start ?? 0) : 0,
          playing: false,
        })),
      });
      return;
    }
    if (syncClock) {
      set({
        slots: slots.map((s) =>
          s.bundle ? { ...s, currentT: clampT(s.bundle, next) } : s,
        ),
      });
    } else {
      const idx = slots.indexOf(leader);
      get().setSlotCurrentT(idx, next);
    }
  },
}));
