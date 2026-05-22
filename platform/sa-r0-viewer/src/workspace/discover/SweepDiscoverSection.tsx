import { SweepCatalogPicker } from "@/replay/SweepCatalogPicker";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  defaultCollapsed?: boolean;
};

export function SweepDiscoverSection({ onLoadError, onLoading, defaultCollapsed = true }: Props) {
  return (
    <CollapsiblePanelSection
      id="discover.sweep_catalog"
      title="Sweep experiments"
      tier="t4"
      defaultCollapsed={defaultCollapsed}
      helper="Monte Carlo sweep families — derived analytics only."
    >
      <SweepCatalogPicker onLoadError={onLoadError} onLoading={onLoading} />
    </CollapsiblePanelSection>
  );
}
