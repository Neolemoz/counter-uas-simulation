import type { Viewer } from "cesium";
import type { ReplaySaBundle } from "@/replay/bundleSchema";
import type { PresentationChapter } from "@/replay/presentation/presentationStore";

export function preparePrintPresentation(): void {
  document.body.classList.add("sandbox-print-presentation");
  window.print();
  window.addEventListener(
    "afterprint",
    () => {
      document.body.classList.remove("sandbox-print-presentation");
    },
    { once: true },
  );
}

export function captureMapCanvasPng(viewer: Viewer | null, filename: string): boolean {
  if (!viewer?.canvas) return false;
  try {
    const dataUrl = viewer.canvas.toDataURL("image/png");
    const a = document.createElement("a");
    a.href = dataUrl;
    a.download = filename;
    a.click();
    return true;
  } catch {
    return false;
  }
}

export function snapshotFilename(
  bundle: ReplaySaBundle | null,
  chapterIndex: number,
): string {
  const pack = bundle?.scenario.catalog_pack_id ?? bundle?.scenario.scenario_id ?? "replay";
  const safe = pack.replace(/[^a-z0-9_]+/gi, "_");
  return `replay_${safe}_ch${chapterIndex}.png`;
}

export async function copyChapterSummaryToClipboard(
  chapter: PresentationChapter | null | undefined,
): Promise<boolean> {
  if (!chapter) return false;
  const text = `${chapter.title}\n\n${chapter.summary}`;
  try {
    await navigator.clipboard.writeText(text);
    return true;
  } catch {
    return false;
  }
}

export function sweepPresentationPacketUrl(sweepId: string | undefined): string | null {
  if (!sweepId) return null;
  return `/demo/sweeps/${sweepId}/reports/sweep_presentation_packet.md`;
}

export function bundleChapterExportText(
  bundle: ReplaySaBundle | null,
  chapterIndex: number,
): string {
  const ch = bundle?.presentation?.chapters?.[chapterIndex];
  if (!ch) return "";
  return `# ${ch.title}\n\n${ch.summary}\n\n---\nExplanatory replay handoff — not operational assessment.`;
}
