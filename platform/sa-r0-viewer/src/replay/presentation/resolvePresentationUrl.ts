import { replayStoryboardSchema } from "./presentationSchema";
import { usePresentationStore } from "./presentationStore";

export async function loadStoryboard(storyboardId: string) {
  const res = await fetch(`/demo/presentations/${storyboardId}.json`);
  if (!res.ok) throw new Error(`storyboard not found: ${storyboardId}`);
  const data: unknown = await res.json();
  return replayStoryboardSchema.parse(data);
}

export async function tryResolvePresentationFromUrl(): Promise<boolean> {
  const params = new URLSearchParams(window.location.search);
  const presentationId = params.get("presentation");
  const walkthrough = params.get("walkthrough");
  const chapterParam = params.get("chapter");
  const chapterIndex = chapterParam != null ? Math.max(0, parseInt(chapterParam, 10) || 0) : 0;

  if (presentationId) {
    const storyboard = await loadStoryboard(presentationId);
    usePresentationStore.getState().enterPresentation(storyboard, chapterIndex);
    return true;
  }

  if (walkthrough === "1") {
    usePresentationStore.getState().enterBundleWalkthrough(chapterIndex);
    return true;
  }

  return false;
}
