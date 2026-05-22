import { useRef, useState } from "react";
import type { Viewer } from "cesium";
import { useClockStore } from "../clockStore";
import { useSweepStore } from "../useSweepStore";
import { usePresentationStore, activeChapter } from "./presentationStore";
import {
  bundleChapterExportText,
  captureMapCanvasPng,
  copyChapterSummaryToClipboard,
  preparePrintPresentation,
  snapshotFilename,
  sweepPresentationPacketUrl,
} from "@/presentation/exportPublicationFrame";
import { sandboxButtons, sandboxSurfaces, sandboxTypography } from "@/theme/sandboxTheme";

type Props = {
  onExit: () => void;
  onToggleFullscreen: () => void;
  mapFullscreen: boolean;
  mapViewer: Viewer | null;
};

export function PresentationModeControls({
  onExit,
  onToggleFullscreen,
  mapFullscreen,
  mapViewer,
}: Props) {
  const bundle = useClockStore((s) => s.bundle);
  const chapters = bundle?.presentation?.chapters ?? [];
  const currentChapterIndex = usePresentationStore((s) => s.currentChapterIndex);
  const setChapterIndex = usePresentationStore((s) => s.setChapterIndex);
  const timelineCompressed = usePresentationStore((s) => s.timelineCompressed);
  const setTimelineCompressed = usePresentationStore((s) => s.setTimelineCompressed);
  const panelSlot = usePresentationStore((s) => s.panelSlot);
  const setPanelSlot = usePresentationStore((s) => s.setPanelSlot);
  const sweepId = useSweepStore((s) => s.sweep?.sweep_id);
  const chapter = activeChapter(bundle, currentChapterIndex);
  const [copyOk, setCopyOk] = useState(false);
  const copyTimer = useRef<number | null>(null);

  const goPrev = () => setChapterIndex(Math.max(0, currentChapterIndex - 1));
  const goNext = () => setChapterIndex(Math.min(chapters.length - 1, currentChapterIndex + 1));

  const onCopy = async () => {
    const ok = await copyChapterSummaryToClipboard(chapter);
    if (ok) {
      setCopyOk(true);
      if (copyTimer.current) window.clearTimeout(copyTimer.current);
      copyTimer.current = window.setTimeout(() => setCopyOk(false), 2000);
    }
  };

  const packetUrl = sweepPresentationPacketUrl(sweepId);
  const chapterText = bundleChapterExportText(bundle, currentChapterIndex);

  return (
    <div className={sandboxSurfaces.publicationBar}>
      <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
        <span className="text-sm font-semibold text-violet-100">Presentation walkthrough</span>
        {chapters.length > 0 && (
          <span className="text-xs tabular-nums text-violet-200/80">
            Chapter {currentChapterIndex + 1} of {chapters.length}
          </span>
        )}
      </div>
      <div className="flex flex-wrap gap-4">
        <div className="flex flex-wrap items-center gap-1.5">
          <span className={sandboxTypography.caption}>Navigate</span>
          <button type="button" className={sandboxButtons.subtle} onClick={goPrev} disabled={currentChapterIndex <= 0}>
            Prev
          </button>
          <button
            type="button"
            className={sandboxButtons.subtle}
            onClick={goNext}
            disabled={currentChapterIndex >= chapters.length - 1}
          >
            Next
          </button>
        </div>
        <div className="flex flex-wrap items-center gap-1.5">
          <span className={sandboxTypography.caption}>View</span>
          <button
            type="button"
            className={sandboxButtons.subtle}
            onClick={() => setTimelineCompressed(!timelineCompressed)}
          >
            {timelineCompressed ? "Expand timeline" : "Chapter timeline"}
          </button>
          <button
            type="button"
            className={sandboxButtons.subtle}
            onClick={() => setPanelSlot(panelSlot === "story" ? "annotations" : "story")}
          >
            {panelSlot === "story" ? "Annotations" : "Story"}
          </button>
          <button type="button" className={sandboxButtons.subtle} onClick={onToggleFullscreen}>
            {mapFullscreen ? "Exit fullscreen" : "Map fullscreen"}
          </button>
        </div>
        <div className="flex flex-wrap items-center gap-1.5">
          <span className={sandboxTypography.caption}>Handoff</span>
          <button type="button" className={sandboxButtons.primary} onClick={() => preparePrintPresentation()}>
            Print layout
          </button>
          <button type="button" className={sandboxButtons.subtle} onClick={() => void onCopy()}>
            {copyOk ? "Copied" : "Copy chapter"}
          </button>
          <button
            type="button"
            className={sandboxButtons.subtle}
            onClick={() => captureMapCanvasPng(mapViewer, snapshotFilename(bundle, currentChapterIndex))}
          >
            Save map frame
          </button>
          {packetUrl ? (
            <a className={sandboxButtons.subtle} href={packetUrl} download>
              Review packet
            </a>
          ) : chapterText ? (
            <a
              className={sandboxButtons.subtle}
              href={`data:text/markdown;charset=utf-8,${encodeURIComponent(chapterText)}`}
              download={`chapter_${currentChapterIndex}.md`}
            >
              Chapter markdown
            </a>
          ) : null}
        </div>
        <button
          type="button"
          className={`${sandboxButtons.subtle} ml-auto`}
          onClick={onExit}
        >
          Exit presentation
        </button>
      </div>
    </div>
  );
}
