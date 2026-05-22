import { CompareView } from "@/replay/compare/CompareView";
import { useCompareStore } from "@/replay/compareStore";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { WorkspaceShell } from "../WorkspaceShell";
import { ExperimentDiscoverRail } from "../discover/ExperimentDiscoverRail";
import { ExperimentLineagePanel } from "@/workflow/ExperimentLineagePanel";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";
import { sandboxTypography } from "@/theme/sandboxTheme";

type Props = {
  onLoadError: (msg: string) => void;
  onLoading: (loading: boolean) => void;
  hooks: ExperimentNavHooks;
};

export function CompareWorkspaceView({ onLoadError, onLoading, hooks }: Props) {
  const compareReady =
    useCompareStore((s) => s.mode === "compare" && s.slotA.bundle && s.slotB.bundle);

  if (compareReady) {
    return <CompareView onLoadError={onLoadError} onLoading={onLoading} hooks={hooks} />;
  }

  const t3 = (
    <>
      <ExperimentDiscoverRail
        segment="compare"
        hooks={hooks}
        onLoadError={onLoadError}
        onLoading={onLoading}
        navigateHooks={hooks}
      />
      <CollapsiblePanelSection
        id="workflow.lineage"
        title="Experiment lineage"
        tier="t4"
        defaultCollapsed={false}
      >
        <ExperimentLineagePanel hooks={hooks} />
      </CollapsiblePanelSection>
    </>
  );

  return (
    <WorkspaceShell
      layoutVariant="segment"
      t3={t3}
      t1={
        <div className="rounded-lg border border-amber-900/25 bg-amber-950/10 p-8 text-center">
          <p className="mb-2 text-base font-medium text-amber-100/90">Compare workspace</p>
          <p className={`mx-auto max-w-lg ${sandboxTypography.body} text-slate-400`}>
            Select a curated pair or two scenario packs to load side-by-side replay comparison.
            Divergence review is explanatory only — not operational benchmarking.
          </p>
        </div>
      }
    />
  );
}
