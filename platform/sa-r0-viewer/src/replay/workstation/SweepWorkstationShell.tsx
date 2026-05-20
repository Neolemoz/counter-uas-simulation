import { ReplayNarrativePanel } from "../analytics/ReplayNarrativePanel";
import { DominantPatternCards } from "./DominantPatternCards";
import { ReplayCohortNav } from "./ReplayCohortNav";
import { ReplayAnomalyBanner } from "./ReplayAnomalyBanner";
import { EventPatternGroupList } from "./EventPatternGroupList";
import { SynthesisSummaryPanel } from "../synthesis/SynthesisSummaryPanel";
import { LinkagePanel } from "../synthesis/LinkagePanel";

export function SweepWorkstationShell() {
  return (
    <div className="flex flex-col gap-3 border-b border-violet-900/30 pb-3">
      <h2 className="text-xs font-semibold uppercase tracking-wide text-violet-300">
        Experiment review workstation
      </h2>
      <ReplayNarrativePanel />
      <SynthesisSummaryPanel />
      <LinkagePanel />
      <ReplayAnomalyBanner />
      <DominantPatternCards />
      <ReplayCohortNav />
      <EventPatternGroupList />
    </div>
  );
}
