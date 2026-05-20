import { useClockStore } from "../clockStore";
import { useSweepStore } from "../useSweepStore";
import { usePresentationStore } from "./presentationStore";

type Props = {
  onExit: () => void;
  onToggleFullscreen: () => void;
  mapFullscreen: boolean;
};

export function PresentationModeControls({ onExit, onToggleFullscreen, mapFullscreen }: Props) {
  const bundle = useClockStore((s) => s.bundle);
  const chapters = bundle?.presentation?.chapters ?? [];
  const currentChapterIndex = usePresentationStore((s) => s.currentChapterIndex);
  const setChapterIndex = usePresentationStore((s) => s.setChapterIndex);
  const timelineCompressed = usePresentationStore((s) => s.timelineCompressed);
  const setTimelineCompressed = usePresentationStore((s) => s.setTimelineCompressed);
  const panelSlot = usePresentationStore((s) => s.panelSlot);
  const setPanelSlot = usePresentationStore((s) => s.setPanelSlot);

  const goPrev = () => setChapterIndex(Math.max(0, currentChapterIndex - 1));
  const goNext = () => setChapterIndex(Math.min(chapters.length - 1, currentChapterIndex + 1));

  return (
    <div className="flex flex-wrap items-center gap-2 rounded border border-violet-900/40 bg-violet-950/20 p-2">
      <span className="text-xs font-semibold uppercase text-violet-200">Presentation mode</span>
      <button
        type="button"
        className="rounded bg-slate-700 px-2 py-1 text-xs hover:bg-slate-600"
        onClick={goPrev}
        disabled={currentChapterIndex <= 0}
      >
        Prev chapter
      </button>
      <button
        type="button"
        className="rounded bg-slate-700 px-2 py-1 text-xs hover:bg-slate-600"
        onClick={goNext}
        disabled={currentChapterIndex >= chapters.length - 1}
      >
        Next chapter
      </button>
      <button
        type="button"
        className="rounded bg-slate-700 px-2 py-1 text-xs hover:bg-slate-600"
        onClick={() => setTimelineCompressed(!timelineCompressed)}
      >
        {timelineCompressed ? "Expand timeline" : "Chapter timeline"}
      </button>
      <button
        type="button"
        className="rounded bg-slate-700 px-2 py-1 text-xs hover:bg-slate-600"
        onClick={() => setPanelSlot(panelSlot === "story" ? "annotations" : "story")}
      >
        Panel: {panelSlot}
      </button>
      <button
        type="button"
        className="rounded bg-slate-700 px-2 py-1 text-xs hover:bg-slate-600"
        onClick={onToggleFullscreen}
      >
        {mapFullscreen ? "Exit map fullscreen" : "Map fullscreen"}
      </button>
      <a
        className="rounded bg-slate-700 px-2 py-1 text-xs hover:bg-slate-600"
        href={`/demo/sweeps/${useSweepStore.getState().sweep?.sweep_id ?? ""}/reports/sweep_presentation_packet.md`}
        download
        onClick={(e) => {
          const sid = useSweepStore.getState().sweep?.sweep_id;
          if (!sid) e.preventDefault();
        }}
      >
        Export review pack
      </a>
      <button
        type="button"
        className="ml-auto rounded border border-slate-600 px-2 py-1 text-xs text-slate-300 hover:bg-slate-800"
        onClick={onExit}
      >
        Exit presentation
      </button>
    </div>
  );
}
