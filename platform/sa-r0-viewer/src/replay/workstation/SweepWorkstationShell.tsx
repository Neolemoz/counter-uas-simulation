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
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { useCorpusStore } from "../corpus/useCorpusStore";
import { useSweepStore } from "../useSweepStore";
import { LinkagePanel } from "../synthesis/LinkagePanel";
import { SweepWalkthroughNav } from "@/workflow/SweepWalkthroughNav";
import { ExperimentLineagePanel } from "@/workflow/ExperimentLineagePanel";

type Props = {
  navigateHooks: NavigateHooks;
  experimentHooks?: ExperimentNavHooks;
};

export function SweepWorkstationShell({ navigateHooks, experimentHooks }: Props) {
  const selectedEntryId = useCorpusStore((s) => s.selectedEntryId);
  const sweep = useSweepStore((s) => s.sweep);
  const entryFromRef = sweep?.corpus_ref?.entry_id ?? null;
  const corpusEntryId = selectedEntryId ?? entryFromRef;
  const hooks = experimentHooks ?? navigateHooks;

  return (
    <div className="flex flex-col gap-3 border-b border-violet-900/30 pb-3">
      <h2 className="text-xs font-semibold uppercase tracking-wide text-violet-300">
        Experiment review workstation
      </h2>
      <ExperimentLineagePanel hooks={hooks} />
      {experimentHooks && <SweepWalkthroughNav hooks={experimentHooks} />}
      <ReplayNarrativePanel />
      <SynthesisSummaryPanel />
      <LinkagePanel hooks={hooks} />
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
