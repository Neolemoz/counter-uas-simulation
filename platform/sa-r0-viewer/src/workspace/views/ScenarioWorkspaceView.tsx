import { useClockStore } from "@/replay/clockStore";
import { loadScenarioCatalog } from "@/replay/loadCatalog";
import { useEffect, useState } from "react";
import type { CatalogPack } from "@/replay/catalogSchema";
import { ScenarioDiscoverSection } from "../discover/ScenarioDiscoverSection";
import { WorkspaceShell } from "../WorkspaceShell";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";
import { ScenarioLoadedSummaryCard } from "../ScenarioLoadedSummaryCard";
import { ExperimentLineagePanel } from "@/workflow/ExperimentLineagePanel";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  hooks: ExperimentNavHooks;
};

export function ScenarioWorkspaceView({ onLoadError, onLoading, hooks }: Props) {
  const bundle = useClockStore((s) => s.bundle);
  const [selectedPack, setSelectedPack] = useState<CatalogPack | null>(null);

  useEffect(() => {
    const packId = bundle?.scenario.catalog_pack_id;
    if (!packId) return;
    void loadScenarioCatalog()
      .then((c) => setSelectedPack(c.packs.find((p) => p.pack_id === packId) ?? null))
      .catch(() => setSelectedPack(null));
  }, [bundle?.scenario.catalog_pack_id]);

  return (
    <WorkspaceShell
      layoutVariant="segment"
      t3={
        <>
          <ScenarioDiscoverSection
            onLoadError={onLoadError}
            onLoading={onLoading}
            hooks={hooks}
            handoffToReplay
          />
          <CollapsiblePanelSection
            id="workflow.lineage"
            title="Experiment lineage"
            tier="t4"
            defaultCollapsed={false}
          >
            <ExperimentLineagePanel
              scenarioPackId={selectedPack?.pack_id ?? bundle?.scenario.catalog_pack_id ?? null}
              hooks={hooks}
            />
          </CollapsiblePanelSection>
        </>
      }
      t1={
        selectedPack ? (
          <ScenarioLoadedSummaryCard pack={selectedPack} bundle={bundle} hooks={hooks} />
        ) : (
          <CollapsiblePanelSection
            id="spatial.map"
            title="Topology preview"
            defaultCollapsed={false}
            helper="Load a scenario pack to open spatial replay review."
          >
            <p className="text-sm text-slate-400">
              Select a scenario from the catalog to load a demo replay bundle. Authoring edits
              remain CLI-side — this segment is read-only fixture discovery.
            </p>
          </CollapsiblePanelSection>
        )
      }
    />
  );
}
