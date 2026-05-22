import { useCallback, useEffect, useMemo, useState } from "react";
import type { Cartesian3 } from "cesium";
import { useCompareStore } from "../compareStore";
import { computeTopologyDiff } from "../topologyDiff";
import { CompareModeControls } from "./CompareModeControls";
import { TopologyComparePanel } from "./TopologyComparePanel";
import { ReplayOutcomeComparePanel } from "./ReplayOutcomeComparePanel";
import { AnnotationsComparePanel } from "./AnnotationsComparePanel";
import { SweepCompareAnalyticsStrip } from "./SweepCompareAnalyticsStrip";
import { CompareSlotMapPane } from "./CompareSlotMapPane";
import { CompareSlotTimeline } from "./CompareSlotTimeline";
import { LayerToggles } from "../LayerToggles";
import { WorkspaceShell } from "@/workspace/WorkspaceShell";
import { CollapsiblePanelSection } from "@/workspace/CollapsiblePanelSection";
import { MockSensorRail } from "@/workspace/MockSensorRail";
import { CompareDiscoverSection } from "@/workspace/discover/CompareDiscoverSection";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { ExperimentLineagePanel } from "@/workflow/ExperimentLineagePanel";
import { ProvenancePanel } from "../ProvenancePanel";

type Props = {
  onLoadError?: (msg: string) => void;
  onLoading?: (loading: boolean) => void;
  hooks?: ExperimentNavHooks;
};

export function CompareView({ onLoadError = () => {}, onLoading = () => {}, hooks }: Props) {
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

  const t3 = (
    <>
      <CompareDiscoverSection onLoadError={onLoadError} onLoading={onLoading} hooks={hooks} />
      {hooks && (
        <CollapsiblePanelSection id="workflow.lineage" title="Experiment lineage" defaultCollapsed>
          <ExperimentLineagePanel hooks={hooks} />
        </CollapsiblePanelSection>
      )}
      {slotA.bundle && (
        <CollapsiblePanelSection id="compare.annotations" title="Slot A provenance" defaultCollapsed>
          <ProvenancePanel bundle={slotA.bundle} />
        </CollapsiblePanelSection>
      )}
      {slotB.bundle && (
        <CollapsiblePanelSection id="narrative.annotations" title="Slot B provenance" defaultCollapsed>
          <ProvenancePanel bundle={slotB.bundle} />
        </CollapsiblePanelSection>
      )}
      <CollapsiblePanelSection id="compare.controls" title="Compare controls" defaultCollapsed={false}>
        <CompareModeControls />
      </CollapsiblePanelSection>
      <CollapsiblePanelSection id="compare.topology" title="Topology divergence" defaultCollapsed={false}>
        <TopologyComparePanel />
      </CollapsiblePanelSection>
      <CollapsiblePanelSection id="compare.outcome" title="Outcome comparison" defaultCollapsed={false}>
        <ReplayOutcomeComparePanel />
      </CollapsiblePanelSection>
      <SweepCompareAnalyticsStrip />
      <CollapsiblePanelSection id="compare.annotations" title="Annotations" defaultCollapsed>
        <AnnotationsComparePanel />
      </CollapsiblePanelSection>
      <CollapsiblePanelSection id="spatial.layers" title="Map layers" defaultCollapsed={false}>
        <LayerToggles />
      </CollapsiblePanelSection>
      {syncClock ? (
        <CollapsiblePanelSection id="temporal.compare" title="Shared timeline" defaultCollapsed={false}>
          <CompareSlotTimeline slot="A" label="Shared timeline (A drives)" />
        </CollapsiblePanelSection>
      ) : (
        <>
          <CollapsiblePanelSection id="temporal.compare" title="Timeline A" defaultCollapsed={false}>
            <CompareSlotTimeline slot="A" label="Timeline A" />
          </CollapsiblePanelSection>
          <CollapsiblePanelSection id="temporal.compare.b" title="Timeline B" defaultCollapsed={false}>
            <CompareSlotTimeline slot="B" label="Timeline B" />
          </CollapsiblePanelSection>
        </>
      )}
    </>
  );

  return (
    <WorkspaceShell
      layoutVariant="compare"
      t3={t3}
      t1={
        <div className="grid min-h-[300px] flex-1 grid-cols-1 gap-3 md:grid-cols-2">
          <div className="flex min-h-0 flex-col overflow-hidden rounded-lg border border-amber-900/25">
            <div className="sandbox-compare-slot-header">
              Replay A — {slotA.bundle?.scenario.title ?? "—"}
            </div>
            <CompareSlotMapPane
              slot="A"
              label="Replay A"
              diffHighlight={diff?.highlight}
              isCameraLeader
              onCameraMatrix={onCameraMatrix}
            />
          </div>
          <div className="flex min-h-0 flex-col overflow-hidden rounded-lg border border-amber-900/25 md:border-l md:border-l-slate-700/50">
            <div className="sandbox-compare-slot-header">
              Replay B — {slotB.bundle?.scenario.title ?? "—"}
            </div>
            <CompareSlotMapPane
              slot="B"
              label="Replay B"
              diffHighlight={diff?.highlight}
              followCamera={cameraFollow}
            />
          </div>
        </div>
      }
      t4t5={<MockSensorRail defaultCollapsed focusLabel={focusSlot} />}
    />
  );
}
