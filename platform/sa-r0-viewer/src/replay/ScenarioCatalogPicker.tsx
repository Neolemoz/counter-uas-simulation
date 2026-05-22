import type { NavigateHooks } from "./corpus/navigateToCorpusEntry";
import { ScenarioDiscoverSection } from "@/workspace/discover/ScenarioDiscoverSection";
import { CorpusDiscoverRail } from "@/workspace/discover/CorpusDiscoverRail";
import { CompareDiscoverSection } from "@/workspace/discover/CompareDiscoverSection";
import { SweepDiscoverSection } from "@/workspace/discover/SweepDiscoverSection";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  navigateHooks: NavigateHooks;
};

/** @deprecated H4 — replaced by workspace segment views and ExperimentDiscoverRail. Kept for reference only. */
/**
 * @deprecated Prefer segment-specific discover rails (H2). Retained for compatibility.
 */
export function ScenarioCatalogPicker({ onLoadError, onLoading, navigateHooks }: Props) {
  return (
    <div className="flex flex-col gap-3">
      <CorpusDiscoverRail navigateHooks={navigateHooks} />
      <CompareDiscoverSection onLoadError={onLoadError} onLoading={onLoading} />
      <SweepDiscoverSection onLoadError={onLoadError} onLoading={onLoading} defaultCollapsed={false} />
      <ScenarioDiscoverSection onLoadError={onLoadError} onLoading={onLoading} />
    </div>
  );
}
