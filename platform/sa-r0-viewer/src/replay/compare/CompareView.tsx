import { useCallback, useEffect, useMemo, useState } from "react";
import type { Cartesian3 } from "cesium";
import { useCompareStore } from "../compareStore";
import { useClockStore } from "../clockStore";
import { computeTopologyDiff } from "../topologyDiff";
import { CompareModeControls } from "./CompareModeControls";
import { TopologyComparePanel } from "./TopologyComparePanel";
import { ReplayOutcomeComparePanel } from "./ReplayOutcomeComparePanel";
import { AnnotationsComparePanel } from "./AnnotationsComparePanel";
import { SweepCompareAnalyticsStrip } from "./SweepCompareAnalyticsStrip";
import { CompareSlotMapPane } from "./CompareSlotMapPane";
import { CompareSlotTimeline } from "./CompareSlotTimeline";
import { LayerToggles } from "../LayerToggles";
import { RadarMockPane } from "@/views/RadarMockPane";
import { EoIrMockPane } from "@/views/EoIrMockPane";
import { InterceptorCameraMockPane } from "@/views/InterceptorCameraMockPane";
import { TelemetryPanel } from "@/views/TelemetryPanel";
import { ThreatAssessmentPanel } from "@/views/ThreatAssessmentPanel";

export function CompareView() {
  const slotA = useCompareStore((s) => s.slotA);
  const slotB = useCompareStore((s) => s.slotB);
  const focusSlot = useCompareStore((s) => s.focusSlot);
  const syncClock = useCompareStore((s) => s.syncClock);
  const playbackMs = useCompareStore((s) => s.playbackMs);
  const tickComparePlayback = useCompareStore((s) => s.tickComparePlayback);
  const [cameraFollow, setCameraFollow] = useState<{
    position: Cartesian3;
    direction: Cartesian3;
    up: Cartesian3;
  } | null>(null);

  const diff = useMemo(() => {
    if (!slotA.bundle || !slotB.bundle) return null;
    return computeTopologyDiff(slotA.bundle, slotB.bundle);
  }, [slotA.bundle, slotB.bundle]);

  const onCameraMatrix = useCallback(
    (state: { position: Cartesian3; direction: Cartesian3; up: Cartesian3 }) => {
      setCameraFollow(state);
    },
    [],
  );

  useEffect(() => {
    const playing = slotA.playing || slotB.playing;
    if (!playing) return;
    const id = window.setInterval(tickComparePlayback, playbackMs);
    return () => window.clearInterval(id);
  }, [slotA.playing, slotB.playing, playbackMs, tickComparePlayback]);

  useEffect(() => {
    const slot = focusSlot === "A" ? slotA : slotB;
    if (!slot.bundle) return;
    useClockStore.setState({
      bundle: slot.bundle,
      currentT: slot.currentT,
      playing: slot.playing,
      selectedEventId: slot.selectedEventId,
      highlightedTrackIds: slot.highlightedTrackIds,
      layers: slot.layers,
      losScope: slot.losScope,
    });
  }, [focusSlot, slotA, slotB]);

  return (
    <main className="grid min-h-0 flex-1 grid-cols-1 gap-3 p-3 lg:grid-cols-12">
      <aside className="flex flex-col gap-3 lg:col-span-3">
        <CompareModeControls />
        <TopologyComparePanel />
        <ReplayOutcomeComparePanel />
        <SweepCompareAnalyticsStrip />
        <AnnotationsComparePanel />
        <LayerToggles />
        {syncClock ? (
          <CompareSlotTimeline slot="A" label="Shared timeline (A drives)" />
        ) : (
          <>
            <CompareSlotTimeline slot="A" label="Timeline A" />
            <CompareSlotTimeline slot="B" label="Timeline B" />
          </>
        )}
      </aside>
      <div className="flex flex-col gap-3 lg:col-span-6">
        <div className="grid min-h-[300px] flex-1 grid-cols-1 gap-2 md:grid-cols-2">
          <CompareSlotMapPane
            slot="A"
            label="Replay A"
            diffHighlight={diff?.highlight}
            isCameraLeader
            onCameraMatrix={onCameraMatrix}
          />
          <CompareSlotMapPane
            slot="B"
            label="Replay B"
            diffHighlight={diff?.highlight}
            followCamera={cameraFollow}
          />
        </div>
      </div>
      <aside className="grid gap-3 lg:col-span-3">
        <p className="text-xs text-slate-500">
          Mock panes: focus {focusSlot} — explanatory mirrors only
        </p>
        <RadarMockPane />
        <EoIrMockPane />
        <InterceptorCameraMockPane />
        <TelemetryPanel />
        <ThreatAssessmentPanel />
      </aside>
    </main>
  );
}
