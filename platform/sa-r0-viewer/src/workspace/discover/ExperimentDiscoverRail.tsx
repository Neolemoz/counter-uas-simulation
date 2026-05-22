import type { WorkspaceSegment } from "../types";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { ScenarioDiscoverSection } from "./ScenarioDiscoverSection";
import { SweepDiscoverSection } from "./SweepDiscoverSection";
import { CompareDiscoverSection } from "./CompareDiscoverSection";
import { OrchestrationStatusPanel } from "@/orchestration/OrchestrationStatusPanel";
import { ValidationStatusPanel } from "@/orchestration/ValidationStatusPanel";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";
import { useClockStore } from "@/replay/clockStore";
import { segmentAllowsPanel } from "../panelRegistry";

type Props = {
  segment: WorkspaceSegment;
  hooks: ExperimentNavHooks;
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  navigateHooks: ExperimentNavHooks;
};

export function ExperimentDiscoverRail({
  segment,
  hooks,
  onLoadError,
  onLoading,
  navigateHooks: _navigateHooks,
}: Props) {
  const bundle = useClockStore((s) => s.bundle);
  const packId = bundle?.scenario.catalog_pack_id ?? null;

  return (
    <>
      {segmentAllowsPanel(segment, "discover.scenario_catalog") && (
        <ScenarioDiscoverSection
          onLoadError={onLoadError}
          onLoading={onLoading}
          hooks={hooks}
          handoffToReplay={segment === "scenario"}
        />
      )}
      {segmentAllowsPanel(segment, "discover.sweep_catalog") && (
        <SweepDiscoverSection
          onLoadError={onLoadError}
          onLoading={onLoading}
          defaultCollapsed={segment !== "corpus"}
        />
      )}
      {segmentAllowsPanel(segment, "discover.validation_status") && packId && (
        <CollapsiblePanelSection
          id="discover.validation_status"
          title="Validation mirror"
          tier="t4"
          defaultCollapsed
        >
          <ValidationStatusPanel scenarioPackId={packId} hooks={hooks} />
        </CollapsiblePanelSection>
      )}
      {segmentAllowsPanel(segment, "discover.job_status") && (
        <CollapsiblePanelSection
          id="discover.job_status"
          title="Experiment queue"
          tier="t4"
          defaultCollapsed={segment === "replay"}
        >
          <OrchestrationStatusPanel hooks={hooks} />
        </CollapsiblePanelSection>
      )}
      {segmentAllowsPanel(segment, "discover.compare_catalog") && (
        <CompareDiscoverSection
          onLoadError={onLoadError}
          onLoading={onLoading}
          hooks={hooks}
        />
      )}
    </>
  );
}
