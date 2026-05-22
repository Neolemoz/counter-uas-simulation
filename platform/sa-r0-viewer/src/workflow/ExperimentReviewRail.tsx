import { CorpusLineageNavPanel } from "@/replay/corpus/CorpusLineageNavPanel";
import { CorpusProvenancePanel } from "@/replay/corpus/CorpusProvenancePanel";
import { CorpusSiblingsStrip } from "@/replay/corpus/CorpusSiblingsStrip";
import { LinkagePanel } from "@/replay/synthesis/LinkagePanel";
import { useCorpusStore } from "@/replay/corpus/useCorpusStore";
import { useClockStore } from "@/replay/clockStore";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { ExperimentLineagePanel } from "./ExperimentLineagePanel";
import { CollapsiblePanelSection } from "@/workspace/CollapsiblePanelSection";
import { CompareDiscoverSection } from "@/workspace/discover/CompareDiscoverSection";

type Props = {
  hooks: ExperimentNavHooks;
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
};

export function ExperimentReviewRail({ hooks, onLoadError, onLoading }: Props) {
  const bundle = useClockStore((s) => s.bundle);
  const selectedEntryId = useCorpusStore((s) => s.selectedEntryId);
  const packId = bundle?.scenario.catalog_pack_id ?? null;

  if (!bundle) return null;

  return (
    <CollapsiblePanelSection
      id="workstation.experiment_review"
      title="Experiment review"
      tier="t4"
      defaultCollapsed={false}
      helper="Lineage, provenance, and compare entry — replay research only."
    >
      <div className="flex flex-col gap-3">
        <ExperimentLineagePanel scenarioPackId={packId} hooks={hooks} />
        <LinkagePanel hooks={hooks} />
        <CorpusSiblingsStrip entryId={selectedEntryId} hooks={hooks} />
        <CorpusLineageNavPanel hooks={hooks} />
        <CorpusProvenancePanel entryId={selectedEntryId} />
        <CompareDiscoverSection onLoadError={onLoadError} onLoading={onLoading} />
      </div>
    </CollapsiblePanelSection>
  );
}
