import { create } from "zustand";
import type { ReplaySaBundle } from "../bundleSchema";
import type { ReplayStoryboard } from "./presentationSchema";

export type NarrativeEmphasis = "ambiguity" | "los" | "topology" | "assignment" | "pacing";

export type PresentationChapter = NonNullable<ReplaySaBundle["presentation"]>["chapters"][number];

type PresentationState = {
  mode: "off" | "presentation";
  storyboard: ReplayStoryboard | null;
  currentSceneIndex: number;
  currentChapterIndex: number;
  spotlightIds: string[];
  narrativeEmphasis: NarrativeEmphasis | null;
  mapFullscreen: boolean;
  timelineCompressed: boolean;
  panelSlot: "story" | "annotations";
  enterPresentation: (storyboard: ReplayStoryboard | null, chapterIndex?: number) => void;
  enterBundleWalkthrough: (chapterIndex?: number) => void;
  exitPresentation: () => void;
  setSceneIndex: (idx: number) => void;
  setChapterIndex: (idx: number) => void;
  setSpotlightIds: (ids: string[]) => void;
  setNarrativeEmphasis: (tag: NarrativeEmphasis | null) => void;
  setMapFullscreen: (v: boolean) => void;
  setTimelineCompressed: (v: boolean) => void;
  setPanelSlot: (slot: "story" | "annotations") => void;
  syncUrlParams: () => void;
};

function replaceParams(params: Record<string, string | null>) {
  if (typeof window === "undefined") return;
  const url = new URL(window.location.href);
  for (const [k, v] of Object.entries(params)) {
    if (v == null) url.searchParams.delete(k);
    else url.searchParams.set(k, v);
  }
  window.history.replaceState({}, "", url.toString());
}

export const usePresentationStore = create<PresentationState>((set, get) => ({
  mode: "off",
  storyboard: null,
  currentSceneIndex: 0,
  currentChapterIndex: 0,
  spotlightIds: [],
  narrativeEmphasis: null,
  mapFullscreen: false,
  timelineCompressed: true,
  panelSlot: "story",
  enterPresentation: (storyboard, chapterIndex = 0) => {
    set({
      mode: "presentation",
      storyboard,
      currentSceneIndex: 0,
      currentChapterIndex: chapterIndex,
      timelineCompressed: true,
      panelSlot: "story",
    });
    replaceParams({
      presentation: storyboard?.storyboard_id ?? "walkthrough",
      chapter: String(chapterIndex),
      walkthrough: storyboard ? null : "1",
    });
  },
  enterBundleWalkthrough: (chapterIndex = 0) => {
    set({
      mode: "presentation",
      storyboard: null,
      currentSceneIndex: 0,
      currentChapterIndex: chapterIndex,
      timelineCompressed: true,
      panelSlot: "story",
    });
    replaceParams({ walkthrough: "1", chapter: String(chapterIndex), presentation: null });
  },
  exitPresentation: () => {
    set({
      mode: "off",
      storyboard: null,
      currentSceneIndex: 0,
      currentChapterIndex: 0,
      spotlightIds: [],
      narrativeEmphasis: null,
      mapFullscreen: false,
    });
    replaceParams({ presentation: null, walkthrough: null, chapter: null });
  },
  setSceneIndex: (idx) => {
    set({ currentSceneIndex: idx });
    const sb = get().storyboard;
    if (sb?.scenes[idx]) {
      const ch = sb.scenes[idx]!.chapter;
      if (ch != null) {
        get().setChapterIndex(ch);
      }
    }
  },
  setChapterIndex: (idx) => {
    set({ currentChapterIndex: idx });
    replaceParams({ chapter: String(idx) });
  },
  setSpotlightIds: (ids) => set({ spotlightIds: ids }),
  setNarrativeEmphasis: (tag) => set({ narrativeEmphasis: tag }),
  setMapFullscreen: (v) => set({ mapFullscreen: v }),
  setTimelineCompressed: (v) => set({ timelineCompressed: v }),
  setPanelSlot: (slot) => set({ panelSlot: slot }),
  syncUrlParams: () => {
    const { mode, storyboard, currentChapterIndex } = get();
    if (mode !== "presentation") return;
    replaceParams({
      presentation: storyboard?.storyboard_id ?? null,
      walkthrough: storyboard ? null : "1",
      chapter: String(currentChapterIndex),
    });
  },
}));

export function activeChapter(bundle: ReplaySaBundle | null, chapterIndex: number): PresentationChapter | null {
  const chapters = bundle?.presentation?.chapters;
  if (!chapters?.length) return null;
  return chapters[Math.min(chapterIndex, chapters.length - 1)] ?? null;
}
