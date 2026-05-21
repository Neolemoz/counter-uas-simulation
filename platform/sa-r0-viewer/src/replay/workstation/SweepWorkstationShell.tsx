import { ReplayNarrativePanel } from "../analytics/ReplayNarrativePanel";
import { DominantPatternCards } from "./DominantPatternCards";
import { ReplayCohortNav } from "./ReplayCohortNav";
import { ReplayAnomalyBanner } from "./ReplayAnomalyBanner";
import { EventPatternGroupList } from "./EventPatternGroupList";
import { SynthesisSummaryPanel } from "../synthesis/SynthesisSummaryPanel";
import { CorpusLineageNavPanel } from "../corpus/CorpusLineageNavPanel";
import { CorpusProvenancePanel } from "../corpus/CorpusProvenancePanel";
import { CorpusSiblingsStrip } from "../corpus/CorpusSiblingsStrip";
import type { NavigateHooks } from "../corpus/navigateToCorpusEntry";
import { useCorpusStore } from "../corpus/useCorpusStore";
import { useSweepStore } from "../useSweepStore";
import { LinkagePanel } from "../synthesis/LinkagePanel";

type Props = {
  navigateHooks: NavigateHooks;
};

export function SweepWorkstationShell({ navigateHooks }: Props) {
  const selectedEntryId = useCorpusStore((s) => s.selectedEntryId);
  const sweep = useSweepStore((s) => s.sweep);
  const entryFromRef = sweep?.corpus_ref?.entry_id ?? null;
  const corpusEntryId = selectedEntryId ?? entryFromRef;

  return (
    <div className="flex flex-col gap-3 border-b border-violet-900/30 pb-3">
      <h2 className="text-xs font-semibold uppercase tracking-wide text-violet-300">
        Experiment review workstation
      </h2>
      <ReplayNarrativePanel />
      <SynthesisSummaryPanel />
      <LinkagePanel />
      <CorpusSiblingsStrip entryId={corpusEntryId} hooks={navigateHooks} />
      <CorpusLineageNavPanel hooks={navigateHooks} />
      <CorpusProvenancePanel entryId={corpusEntryId} />
      <ReplayAnomalyBanner />
      <DominantPatternCards />
      <ReplayCohortNav />
      <EventPatternGroupList />
    </div>
  );
}
