import { useEffect, useState } from "react";
import { useClockStore } from "@/replay/clockStore";
import { usePresentationStore } from "@/replay/presentation/presentationStore";
import { loadStoryboard } from "@/replay/presentation/resolvePresentationUrl";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { WorkspaceShell } from "../WorkspaceShell";
import { CollapsiblePanelSection } from "../CollapsiblePanelSection";
import { ExperimentLineagePanel } from "@/workflow/ExperimentLineagePanel";
import { sandboxSurfaces, sandboxTypography } from "@/theme/sandboxTheme";
import { preparePrintPresentation } from "@/presentation/exportPublicationFrame";

type StoryboardEntry = {
  storyboard_id: string;
  title: string;
  estimated_minutes?: number;
};

type Props = {
  hooks: ExperimentNavHooks;
};

export function ReportWorkspaceView({ hooks }: Props) {
  const bundle = useClockStore((s) => s.bundle);
  const enterPresentation = usePresentationStore((s) => s.enterPresentation);
  const enterBundleWalkthrough = usePresentationStore((s) => s.enterBundleWalkthrough);
  const [storyboards, setStoryboards] = useState<StoryboardEntry[]>([]);

  useEffect(() => {
    fetch("/demo/presentations/index.json")
      .then((r) => r.json())
      .then((data: { storyboards?: StoryboardEntry[] }) =>
        setStoryboards(data.storyboards ?? []),
      )
      .catch(() => setStoryboards([]));
  }, []);

  const t3 = (
    <>
      <CollapsiblePanelSection
        id="presentation.story"
        title="Presentation catalog"
        defaultCollapsed={false}
        helper="Guided replay review — derived summaries only."
      >
        <ul className="space-y-2">
          {storyboards.map((sb) => (
            <li key={sb.storyboard_id}>
              <button
                type="button"
                className={sandboxSurfaces.storyCard}
                onClick={() =>
                  void loadStoryboard(sb.storyboard_id).then((board) => enterPresentation(board, 0))
                }
              >
                <span className="block text-sm font-medium text-slate-100">{sb.title}</span>
                <span className={`mt-1 block ${sandboxTypography.monoRef}`}>
                  {sb.storyboard_id}
                  {sb.estimated_minutes != null ? ` · ~${sb.estimated_minutes} min` : ""}
                </span>
              </button>
            </li>
          ))}
        </ul>
      </CollapsiblePanelSection>
      <CollapsiblePanelSection
        id="presentation.controls"
        title="Bundle walkthrough"
        defaultCollapsed={false}
      >
        {bundle?.presentation?.chapters?.length ? (
          <div className="space-y-2">
            <button
              type="button"
              className="w-full rounded-lg border border-violet-800/40 bg-violet-950/30 py-2.5 text-sm text-violet-100 transition-colors hover:bg-violet-900/35"
              onClick={() => enterBundleWalkthrough(0)}
            >
              Start walkthrough ({bundle.presentation.chapters.length} chapters)
            </button>
            <p className={sandboxTypography.caption}>
              After entering presentation, use Print layout or Save map frame from the toolbar.
            </p>
            <button
              type="button"
              className="text-xs text-violet-300/80 underline hover:text-violet-200"
              onClick={() => preparePrintPresentation()}
            >
              Preview print layout (current view)
            </button>
          </div>
        ) : (
          <p className={sandboxTypography.caption}>
            Load a replay bundle with presentation chapters, or pick a storyboard above.
          </p>
        )}
      </CollapsiblePanelSection>
      <CollapsiblePanelSection id="workflow.lineage" title="Experiment lineage" defaultCollapsed>
        <ExperimentLineagePanel
          scenarioPackId={bundle?.scenario.catalog_pack_id ?? null}
          hooks={hooks}
        />
      </CollapsiblePanelSection>
    </>
  );

  return (
    <WorkspaceShell
      layoutVariant="segment"
      t3={t3}
      t1={
        <div className="rounded-lg border border-violet-900/25 bg-violet-950/10 p-8 text-center">
          <p className="mb-2 text-base font-medium text-violet-100">Report and presentation</p>
          <p className={`mx-auto max-w-lg ${sandboxTypography.body} text-slate-400`}>
            Choose a storyboard or start a bundle walkthrough. Exports are explanatory research
            handoffs — not certification or readiness assessment.
          </p>
        </div>
      }
    />
  );
}
