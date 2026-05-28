import { useCallback, useMemo } from "react";
import { PanelShell } from "@/components/GovernanceChrome";
import { BANNER_EXPERIMENT_V2 } from "@/governance/banners";
import { getCohort } from "./cohortIndexStore";
import { ExperimentCohortNavigator } from "./ExperimentCohortNavigator";
import { ExperimentCompareStagePanel } from "./ExperimentCompareStagePanel";
import {
  ExperimentReportDockPanel,
  type ReportDockPresence,
} from "./ExperimentReportDockPanel";
import { ExperimentUnifiedReviewPanel } from "./ExperimentUnifiedReviewPanel";
import { ExperimentWorkbenchV3Shell } from "./ExperimentWorkbenchV3Shell";
import { MultiManifestDiffTable } from "./MultiManifestDiffTable";
import type { CompareModeId, UnifiedReviewStepId } from "./experimentUnifiedReview";
import type { ExperimentManifest } from "./experimentSchema";
import {
  saveWorkbenchV2State,
  selectPrimaryManifestRef,
  selectSecondaryManifestRef,
  type WorkbenchV2State,
} from "./workbenchV2State";

export function ExperimentWorkbenchV2Shell({
  v2State,
  onV2StateChange,
  manifest,
  reportPresence,
  dockPreviews,
  onImportDockSlot,
  onExportDockSlot,
  onActivateStep,
  onContinuityRunId,
  onSyncComparePinned,
  onApplyCompareMode,
  dockPacketTabFocus,
}: {
  v2State: WorkbenchV2State;
  onV2StateChange: (state: WorkbenchV2State) => void;
  manifest: ExperimentManifest;
  reportPresence: ReportDockPresence;
  dockPreviews: Partial<Record<string, string>>;
  onImportDockSlot: (slotId: string, text: string) => string | null;
  onExportDockSlot: (slotId: string) => string | null;
  onActivateStep: (step: UnifiedReviewStepId) => void;
  onContinuityRunId: (runId: string) => void;
  onSyncComparePinned: (runA: string | null, runB: string | null) => void;
  onApplyCompareMode: (mode: CompareModeId) => void;
  dockPacketTabFocus?: number;
}) {
  const activeCohort = useMemo(
    () => (v2State.active_cohort_id ? getCohort(v2State.active_cohort_id) : null),
    [v2State.active_cohort_id],
  );

  const showMultiManifestDiff = v2State.compare_mode === "multi_manifest_diff";

  const openManifestRef = useCallback(
    (manifestRef: string, role: "primary" | "secondary") => {
      const next =
        role === "primary"
          ? selectPrimaryManifestRef(v2State, manifestRef)
          : selectSecondaryManifestRef(v2State, manifestRef);
      saveWorkbenchV2State(next);
      onV2StateChange(next);
    },
    [v2State, onV2StateChange],
  );

  return (
    <details className="mb-4" data-testid="experiment-workbench-v2">
      <summary className="cursor-pointer text-sm font-medium text-slate-300">
        Experiment workbench v2 (cohort + unified review)
      </summary>
      <div className="mt-2 space-y-3">
        <PanelShell title="Experiment workbench v2">
          <ExperimentWorkbenchV3Shell
            v2State={v2State}
            cohort={activeCohort}
            loadedManifest={manifest}
            onV2StateChange={onV2StateChange}
          >
            <p className="mb-2 text-[10px] text-amber-200/90">{BANNER_EXPERIMENT_V2}</p>
            <div className="grid gap-3 lg:grid-cols-3">
              <ExperimentCohortNavigator v2State={v2State} onV2StateChange={onV2StateChange} />
              <ExperimentUnifiedReviewPanel
                v2State={v2State}
                onV2StateChange={onV2StateChange}
                manifest={manifest}
                presence={reportPresence}
                runs={manifest.runs}
                onActivateStep={onActivateStep}
                onContinuityRunId={onContinuityRunId}
                onSyncComparePinned={onSyncComparePinned}
                packetTabEverFocused={dockPacketTabFocus != null && dockPacketTabFocus > 0}
              />
              <ExperimentReportDockPanel
                v2State={v2State}
                manifest={manifest}
                presence={reportPresence}
                previews={dockPreviews}
                onImportSlot={onImportDockSlot}
                onExportSlot={onExportDockSlot}
                packetTabFocusToken={dockPacketTabFocus}
                cohortLabel={activeCohort?.label ?? null}
                cohort={activeCohort}
              />
            </div>
            <div className="mt-3 space-y-3 border-t border-slate-800/80 pt-3">
              <p className="text-[10px] font-semibold uppercase tracking-wide text-slate-500">
                Compare stage
              </p>
              <ExperimentCompareStagePanel
                v2State={v2State}
                onV2StateChange={onV2StateChange}
                onApplyCompareMode={onApplyCompareMode}
                cohort={activeCohort}
              />
              {showMultiManifestDiff && (
                <MultiManifestDiffTable
                  cohort={activeCohort}
                  primaryManifestRef={v2State.primary_manifest_ref}
                  secondaryManifestRef={v2State.secondary_manifest_ref}
                  loadedManifest={manifest}
                  onOpenManifestRef={openManifestRef}
                />
              )}
            </div>
          </ExperimentWorkbenchV3Shell>
        </PanelShell>
      </div>
    </details>
  );
}
