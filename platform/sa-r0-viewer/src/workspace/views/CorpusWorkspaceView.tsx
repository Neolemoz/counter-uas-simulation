import { useEffect } from "react";
import { useClockStore } from "@/replay/clockStore";
import { StrategicMapPane } from "@/views/StrategicMapPane";
import type { NavigateHooks } from "@/replay/corpus/navigateToCorpusEntry";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { ExperimentDiscoverRail } from "../discover/ExperimentDiscoverRail";
import { WorkspaceShell } from "../WorkspaceShell";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";
import { ExperimentLineagePanel } from "@/workflow/ExperimentLineagePanel";
import { CorpusBrowserPanel } from "@/replay/corpus/CorpusBrowserPanel";
import { CorpusEvolutionPanel } from "@/replay/corpus/CorpusEvolutionPanel";
import { CorpusLineageNavPanel } from "@/replay/corpus/CorpusLineageNavPanel";
import { CorpusProvenancePanel } from "@/replay/corpus/CorpusProvenancePanel";
import { useCorpusStore } from "@/replay/corpus/useCorpusStore";
import { FederationBrowserPanel } from "@/replay/federation/FederationBrowserPanel";
import { FederationContinuityPanel } from "@/replay/federation/FederationContinuityPanel";
import { FederationLineagePanel } from "@/replay/federation/FederationLineagePanel";
import { FederationProvenancePanel } from "@/replay/federation/FederationProvenancePanel";
import { FederationPublicationCollectionPanel } from "@/replay/federation/FederationPublicationCollectionPanel";
import {
  readFederationFromUrl,
  useFederationStore,
} from "@/replay/federation/useFederationStore";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  navigateHooks: NavigateHooks;
  experimentHooks: ExperimentNavHooks;
};

export function CorpusWorkspaceView({
  onLoadError,
  onLoading,
  navigateHooks,
  experimentHooks,
}: Props) {
  const bundle = useClockStore((s) => s.bundle);
  const entryId = useCorpusStore((s) => s.selectedEntryId);
  const setFederationId = useFederationStore((s) => s.setFederationId);
  const setCorpusGroupId = useFederationStore((s) => s.setCorpusGroupId);
  const setHighlightedLineageRef = useFederationStore((s) => s.setHighlightedLineageRef);

  useEffect(() => {
    const url = readFederationFromUrl();
    if (url.federationId) setFederationId(url.federationId);
    if (url.corpusGroupId) setCorpusGroupId(url.corpusGroupId);
    if (url.federationLineageRef) setHighlightedLineageRef(url.federationLineageRef);
  }, [setFederationId, setCorpusGroupId, setHighlightedLineageRef]);

  return (
    <WorkspaceShell
      layoutVariant="segment"
      t3={
        <>
          <CollapsiblePanelSection
            id="corpus.federation"
            title="Federation registry"
            defaultCollapsed
            helper="Multi-corpus offline registry — read-only, not cloud sync."
          >
            <FederationBrowserPanel />
          </CollapsiblePanelSection>
          <CollapsiblePanelSection
            id="corpus.federation.lineage"
            title="Federation lineage"
            defaultCollapsed
          >
            <FederationLineagePanel />
          </CollapsiblePanelSection>
          <CollapsiblePanelSection
            id="corpus.federation.provenance"
            title="Federation integrity"
            defaultCollapsed
          >
            <FederationProvenancePanel />
          </CollapsiblePanelSection>
          <CollapsiblePanelSection
            id="corpus.federation.publications"
            title="Publication collections"
            defaultCollapsed
          >
            <FederationPublicationCollectionPanel />
          </CollapsiblePanelSection>
          <CollapsiblePanelSection
            id="corpus.federation.continuity"
            title="Federation continuity"
            defaultCollapsed
          >
            <FederationContinuityPanel />
          </CollapsiblePanelSection>
          <CollapsiblePanelSection
            id="corpus.browser"
            title="Corpus browser"
            defaultCollapsed={false}
            helper="Index mirror — not operational deployment catalog."
          >
            <CorpusBrowserPanel hooks={navigateHooks} />
          </CollapsiblePanelSection>
          <CollapsiblePanelSection id="corpus.evolution" title="Evolution chronology" defaultCollapsed>
            <CorpusEvolutionPanel hooks={navigateHooks} />
          </CollapsiblePanelSection>
          <CollapsiblePanelSection id="corpus.lineage" title="Lineage navigation" defaultCollapsed>
            <CorpusLineageNavPanel hooks={navigateHooks} />
          </CollapsiblePanelSection>
          <CollapsiblePanelSection
            id="corpus.provenance"
            title="Provenance and drift"
            defaultCollapsed
          >
            <CorpusProvenancePanel entryId={entryId} />
          </CollapsiblePanelSection>
          <ExperimentDiscoverRail
            segment="corpus"
            hooks={experimentHooks}
            onLoadError={onLoadError}
            onLoading={onLoading}
            navigateHooks={navigateHooks}
          />
          <CollapsiblePanelSection id="workflow.lineage" title="Experiment lineage" defaultCollapsed>
            <ExperimentLineagePanel
              scenarioPackId={bundle?.scenario.catalog_pack_id ?? null}
              hooks={experimentHooks}
            />
          </CollapsiblePanelSection>
        </>
      }
      t1={
        bundle ? (
          <StrategicMapPane bundle={bundle} />
        ) : (
          <p className="rounded border border-slate-700 bg-slate-900/80 p-6 text-center text-sm text-slate-500">
            Select a corpus entry to load its linked replay bundle for spatial context.
          </p>
        )
      }
    />
  );
}
