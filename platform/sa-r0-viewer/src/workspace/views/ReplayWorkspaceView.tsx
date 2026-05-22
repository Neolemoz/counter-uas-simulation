import type { ReplaySaBundle } from "@/replay/bundleSchema";
import { MetadataPanel } from "@/replay/MetadataPanel";
import { LayerToggles } from "@/replay/LayerToggles";
import { TimelineScrubber } from "@/replay/timeline/TimelineScrubber";
import { NarrativeTimeline } from "@/replay/narrative/NarrativeTimeline";
import { AnnotationsPanel } from "@/replay/narrative/AnnotationsPanel";
import { SweepMetadataPanel } from "@/replay/SweepMetadataPanel";
import { OutcomeDistributionPanel } from "@/replay/analytics/OutcomeDistributionPanel";
import { LosDegradationSummary } from "@/replay/analytics/LosDegradationSummary";
import { ReplayVariabilityPanel } from "@/replay/analytics/ReplayVariabilityPanel";
import { ReplayClusterSummaryPanel } from "@/replay/analytics/ReplayClusterSummaryPanel";
import { MatchedSeedPanel } from "@/replay/analytics/MatchedSeedPanel";
import { useSweepStore } from "@/replay/useSweepStore";
import { SweepWorkstationShell } from "@/replay/workstation/SweepWorkstationShell";
import { StrategicMapPane } from "@/views/StrategicMapPane";
import type { NavigateHooks } from "@/replay/corpus/navigateToCorpusEntry";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { WorkspaceShell } from "../WorkspaceShell";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";
import { MockSensorRail } from "../MockSensorRail";
import { ExperimentDiscoverRail } from "../discover/ExperimentDiscoverRail";
import { ExperimentReviewRail } from "@/workflow/ExperimentReviewRail";
import { usePresentationStore } from "@/replay/presentation/presentationStore";
import { useWorkspaceSegmentStore } from "../workspaceSegmentStore";

type Props = {
  bundle: ReplaySaBundle;
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  navigateHooks: NavigateHooks;
  experimentHooks: ExperimentNavHooks;
};

export function ReplayWorkspaceView({
  bundle,
  onLoadError,
  onLoading,
  navigateHooks,
  experimentHooks,
}: Props) {
  const sweepMode = useSweepStore((s) => s.mode === "sweep");
  const enterWalkthrough = usePresentationStore((s) => s.enterBundleWalkthrough);
  const setSegment = useWorkspaceSegmentStore((s) => s.setUserSegment);

  const t3 = (
    <>
      <ExperimentDiscoverRail
        segment="replay"
        hooks={experimentHooks}
        onLoadError={onLoadError}
        onLoading={onLoading}
        navigateHooks={navigateHooks}
      />
      {sweepMode ? (
        <>
          <SweepMetadataPanel />
          <CollapsiblePanelSection
            id="workstation.sweep"
            title="Experiment workstation"
            tier="t4"
            defaultCollapsed={false}
          >
            <SweepWorkstationShell navigateHooks={navigateHooks} experimentHooks={experimentHooks} />
          </CollapsiblePanelSection>
        </>
      ) : (
        <ExperimentReviewRail
          hooks={experimentHooks}
          onLoadError={onLoadError}
          onLoading={onLoading}
        />
      )}
      <CollapsiblePanelSection id="metadata.bundle" title="Bundle metadata" defaultCollapsed={false}>
        <MetadataPanel bundle={bundle} />
      </CollapsiblePanelSection>
      {bundle.presentation?.chapters?.length ? (
        <CollapsiblePanelSection
          id="presentation.controls"
          title="Presentation entry"
          tier="t4"
          defaultCollapsed
        >
          <button
            type="button"
            className="w-full rounded bg-violet-900/50 py-2 text-xs text-violet-100 hover:bg-violet-900/70"
            onClick={() => {
              enterWalkthrough(0);
              setSegment("report");
            }}
          >
            Start bundle walkthrough ({bundle.presentation.chapters.length} chapters)
          </button>
        </CollapsiblePanelSection>
      ) : null}
      {sweepMode && (
        <CollapsiblePanelSection
          id="analytics.sweep"
          title="Sweep analytics"
          tier="t4"
          defaultCollapsed
        >
          <OutcomeDistributionPanel />
          <div className="mt-2 grid gap-2">
            <LosDegradationSummary />
            <ReplayVariabilityPanel />
            <ReplayClusterSummaryPanel />
            <MatchedSeedPanel />
          </div>
        </CollapsiblePanelSection>
      )}
      <CollapsiblePanelSection id="spatial.layers" title="Map layers" defaultCollapsed={false}>
        <LayerToggles />
      </CollapsiblePanelSection>
      <CollapsiblePanelSection id="temporal.scrubber" title="Timeline" defaultCollapsed={false}>
        <TimelineScrubber />
        <p className="mt-1 text-[10px] text-slate-600">
          Replay clock — explanatory timeline only, not live sensor time.
        </p>
      </CollapsiblePanelSection>
      <CollapsiblePanelSection id="narrative.annotations" title="Annotations" defaultCollapsed>
        <AnnotationsPanel />
      </CollapsiblePanelSection>
    </>
  );

  return (
    <WorkspaceShell
      layoutVariant="segment"
      t3={t3}
      t1={
        <>
          <StrategicMapPane bundle={bundle} />
        </>
      }
      t2={<NarrativeTimeline />}
      t4t5={<MockSensorRail defaultCollapsed />}
    />
  );
}
