import { useEffect, useRef } from "react";
import { StrategicMapPane } from "@/views/StrategicMapPane";
import { useClockStore } from "../clockStore";
import { useSweepStore } from "../useSweepStore";
import { AnnotationsPanel } from "../narrative/AnnotationsPanel";
import { TimelineScrubber } from "../timeline/TimelineScrubber";
import { PresentationLayoutShell } from "./PresentationLayoutShell";
import { ChapterNavRail } from "./ChapterNavRail";
import { PresentationModeControls } from "./PresentationModeControls";
import { ReplayStorytellingPanel } from "./ReplayStorytellingPanel";
import { PresentationCognitionPanel } from "./PresentationCognitionPanel";
import {
  activeChapter,
  usePresentationStore,
} from "./presentationStore";
import {
  applyChapterClock,
  applyChapterToLayers,
  chapterLosScope,
  chapterSpatialDeclutter,
} from "./applyPresentationChapter";

export function PresentationView() {
  const bundle = useClockStore((s) => s.bundle);
  const setCurrentT = useClockStore((s) => s.setCurrentT);
  const setSelectedEventId = useClockStore((s) => s.setSelectedEventId);
  const setLosScope = useClockStore((s) => s.setLosScope);
  const exitPresentation = usePresentationStore((s) => s.exitPresentation);
  const currentChapterIndex = usePresentationStore((s) => s.currentChapterIndex);
  const setSpotlightIds = usePresentationStore((s) => s.setSpotlightIds);
  const setNarrativeEmphasis = usePresentationStore((s) => s.setNarrativeEmphasis);
  const mapFullscreen = usePresentationStore((s) => s.mapFullscreen);
  const setMapFullscreen = usePresentationStore((s) => s.setMapFullscreen);
  const panelSlot = usePresentationStore((s) => s.panelSlot);
  const spotlightIds = usePresentationStore((s) => s.spotlightIds);
  const timelineCompressed = usePresentationStore((s) => s.timelineCompressed);
  const setSpatialDeclutter = useSweepStore((s) => s.setSpatialDeclutter);
  const mapRef = useRef<HTMLDivElement>(null);

  const chapter = activeChapter(bundle, currentChapterIndex);

  useEffect(() => {
    if (!bundle || !chapter) return;
    const { t, eventId } = applyChapterClock(bundle, chapter);
    setCurrentT(t);
    setSelectedEventId(eventId);
    setSpotlightIds(chapter.spotlight_annotation_ids ?? []);
    setNarrativeEmphasis(chapter.narrative_emphasis ?? null);
    setSpatialDeclutter(chapterSpatialDeclutter(chapter));
    const presets = useClockStore.getState();
    const losScope = chapterLosScope(chapter, presets.losScope);
    setLosScope(losScope);
    useClockStore.setState({
      layers: applyChapterToLayers(presets.layers, chapter),
    });
    useClockStore.getState().requestFitReplay();
  }, [bundle, chapter, currentChapterIndex, setCurrentT, setSelectedEventId, setSpotlightIds, setNarrativeEmphasis, setLosScope, setSpatialDeclutter]);

  const toggleFullscreen = async () => {
    const el = mapRef.current;
    if (!el) return;
    if (!document.fullscreenElement) {
      await el.requestFullscreen?.();
      setMapFullscreen(true);
    } else {
      await document.exitFullscreen?.();
      setMapFullscreen(false);
    }
  };

  if (!bundle) {
    return (
      <p className="p-8 text-center text-slate-500">
        Presentation mode requires a loaded replay bundle.
      </p>
    );
  }

  const leftPanel =
    panelSlot === "story" ? (
      <>
        <ReplayStorytellingPanel bundle={bundle} />
        <PresentationCognitionPanel />
      </>
    ) : (
      <AnnotationsPanel spotlightIds={spotlightIds} />
    );

  return (
    <PresentationLayoutShell
      left={mapFullscreen ? undefined : leftPanel}
      center={
        <>
          <PresentationModeControls
            onExit={exitPresentation}
            onToggleFullscreen={() => void toggleFullscreen()}
            mapFullscreen={mapFullscreen}
          />
          <ChapterNavRail />
          <div ref={mapRef} className={mapFullscreen ? "h-screen w-full bg-slate-950" : "min-h-[420px] flex-1"}>
            <StrategicMapPane bundle={bundle} presentationDimming />
          </div>
          <TimelineScrubber compressed={timelineCompressed} chapter={chapter} />
        </>
      }
    />
  );
}
