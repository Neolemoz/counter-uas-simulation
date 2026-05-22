import { CorpusBrowserPanel } from "@/replay/corpus/CorpusBrowserPanel";
import { CorpusEvolutionPanel } from "@/replay/corpus/CorpusEvolutionPanel";
import { CorpusLineageNavPanel } from "@/replay/corpus/CorpusLineageNavPanel";
import { CorpusProvenancePanel } from "@/replay/corpus/CorpusProvenancePanel";
import { useCorpusStore } from "@/replay/corpus/useCorpusStore";
import type { NavigateHooks } from "@/replay/corpus/navigateToCorpusEntry";
import { OrchestrationStatusPanel } from "@/orchestration/OrchestrationStatusPanel";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";

type Props = {
  navigateHooks: NavigateHooks;
};

export function CorpusDiscoverRail({ navigateHooks }: Props) {
  const entryId = useCorpusStore((s) => s.selectedEntryId);

  return (
    <>
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
        helper="Corpus audit mirror — not deployment authority."
      >
        <CorpusProvenancePanel entryId={entryId} />
      </CollapsiblePanelSection>
      <CollapsiblePanelSection id="discover.job_status" title="Experiment queue" defaultCollapsed>
        <OrchestrationStatusPanel />
      </CollapsiblePanelSection>
    </>
  );
}
