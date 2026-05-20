import { describe, expect, it } from "vitest";
import { activeChapter, usePresentationStore } from "./presentationStore";
import type { ReplaySaBundle } from "../bundleSchema";

const minimalBundle = {
  presentation: {
    walkthrough_id: "test_walkthrough",
    chapters: [
      {
        chapter_id: "ingress",
        title: "Ingress",
        t_start: 0,
        t_end: 10,
        summary: "Ingress chapter.",
      },
      {
        chapter_id: "outcome",
        title: "Outcome",
        t_start: 11,
        t_end: 20,
        summary: "Outcome chapter.",
      },
    ],
  },
} as ReplaySaBundle;

describe("presentationStore", () => {
  it("enters and exits presentation mode", () => {
    usePresentationStore.getState().exitPresentation();
    expect(usePresentationStore.getState().mode).toBe("off");
    usePresentationStore.getState().enterBundleWalkthrough(1);
    expect(usePresentationStore.getState().mode).toBe("presentation");
    expect(usePresentationStore.getState().currentChapterIndex).toBe(1);
    usePresentationStore.getState().exitPresentation();
    expect(usePresentationStore.getState().mode).toBe("off");
  });

  it("resolves active chapter", () => {
    expect(activeChapter(minimalBundle, 0)?.chapter_id).toBe("ingress");
    expect(activeChapter(minimalBundle, 5)?.chapter_id).toBe("outcome");
  });
});
