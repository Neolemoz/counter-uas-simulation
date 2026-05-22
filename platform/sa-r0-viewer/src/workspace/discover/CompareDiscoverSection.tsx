import { CompareCatalogSection } from "@/replay/compare/CompareCatalogSection";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  hooks?: ExperimentNavHooks;
};

export function CompareDiscoverSection({ onLoadError, onLoading, hooks }: Props) {
  return (
    <CollapsiblePanelSection
      id="discover.compare_catalog"
      title="Compare pairs"
      defaultCollapsed={false}
      helper="Curated A/B pairs — divergence review, not ranking."
    >
      <CompareCatalogSection onLoadError={onLoadError} onLoading={onLoading} hooks={hooks} />
    </CollapsiblePanelSection>
  );
}
