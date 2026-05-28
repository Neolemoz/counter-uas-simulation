import { PanelShell } from "@/components/GovernanceChrome";
import type { AdvisoryStatus } from "@/handoff/advisoryTypes";
import { ExperimentExtendedComparePanel } from "./ExperimentExtendedComparePanel";
import { ExperimentFidelityCompareStrip } from "./ExperimentFidelityCompareStrip";
import { ExperimentFilterBar } from "./ExperimentFilterBar";
import { ExperimentHandoffEligibilityStrip } from "./ExperimentHandoffEligibilityStrip";
import { ExperimentImportAdvisoryStrip } from "./ExperimentImportAdvisoryStrip";
import { ExperimentMatrixPanel } from "./ExperimentMatrixPanel";
import { ExperimentRepeatabilityTrendStrip } from "./ExperimentRepeatabilityTrendStrip";
import type { F5Filters } from "./experimentF5UiHelpers";
import type {
  ExperimentAnalyticsReport,
  ExperimentFidelityMetricsReport,
  ExperimentManifest,
  ExperimentMetricsReport,
  ExperimentRun,
} from "./experimentSchema";

type FilterOptions = {
  experiment_class: string[];
  tactical_mode: string[];
  terrain_preset: string[];
  visibility_context: string[];
};

export function ExperimentF5MetricsSection({
  manifest,
  metricsReport,
  fidelityReport,
  analyticsReport,
  filteredRuns,
  f5Filters,
  onF5FiltersChange,
  filterOptions,
  fidelityCouplingPresent,
  matrixAxisRow,
  matrixAxisCol,
  onMatrixAxisRowChange,
  onMatrixAxisColChange,
  extendedCompareRunIds,
  onExtendedCompareRunIdsChange,
  maintainerAckPoseReviewed,
  onMaintainerAckPoseReviewedChange,
  workbenchAdvisoryStatus,
  metricsCliHint,
  fidelityMetricsCliHint,
  onImportMetrics,
  onExportMetrics,
  onImportFidelityMetrics,
  onExportFidelityMetrics,
  onRefreshMetrics,
  onRefreshFidelityMetrics,
}: {
  manifest: ExperimentManifest;
  metricsReport: ExperimentMetricsReport | null;
  fidelityReport: ExperimentFidelityMetricsReport | null;
  analyticsReport: ExperimentAnalyticsReport;
  filteredRuns: ExperimentRun[];
  f5Filters: F5Filters;
  onF5FiltersChange: (filters: F5Filters) => void;
  filterOptions: FilterOptions;
  fidelityCouplingPresent: boolean;
  matrixAxisRow: string;
  matrixAxisCol: string;
  onMatrixAxisRowChange: (value: string) => void;
  onMatrixAxisColChange: (value: string) => void;
  extendedCompareRunIds: string[];
  onExtendedCompareRunIdsChange: (
    ids: string[] | ((prev: string[]) => string[]),
  ) => void;
  maintainerAckPoseReviewed: boolean;
  onMaintainerAckPoseReviewedChange: (value: boolean) => void;
  workbenchAdvisoryStatus: AdvisoryStatus | null;
  metricsCliHint: string;
  fidelityMetricsCliHint: string;
  onImportMetrics: () => void;
  onExportMetrics: () => void;
  onImportFidelityMetrics: () => void;
  onExportFidelityMetrics: () => void;
  onRefreshMetrics: () => void;
  onRefreshFidelityMetrics: () => void;
}) {
  return (
    <PanelShell title="Advanced experiment metrics (F5)">
      {manifest.runs.length === 0 ? (
        <p className="text-xs text-slate-500">
          Pin or import manifest runs to derive advanced metrics.
        </p>
      ) : metricsReport ? (
        <div className="space-y-4">
          <div className="flex flex-wrap gap-2">
            <p className="w-full font-mono text-[10px] text-slate-500">{metricsCliHint}</p>
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
              onClick={onImportMetrics}
            >
              Import metrics report
            </button>
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
              onClick={onRefreshMetrics}
            >
              Refresh from manifest
            </button>
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
              onClick={onExportMetrics}
            >
              Export metrics report
            </button>
          </div>
          <ExperimentFilterBar
            filters={f5Filters}
            onChange={onF5FiltersChange}
            options={filterOptions}
          />
          {metricsReport.experiment_class === "repeatability_sweep" && (
            <ExperimentRepeatabilityTrendStrip
              manifest={manifest}
              metricsReport={metricsReport}
              f1PerRun={analyticsReport.per_run}
            />
          )}
          {fidelityCouplingPresent && fidelityReport && (
            <>
              <div className="flex flex-wrap gap-2">
                <p className="w-full font-mono text-[10px] text-slate-500">
                  {fidelityMetricsCliHint}
                </p>
                <button
                  type="button"
                  className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
                  onClick={onImportFidelityMetrics}
                >
                  Import fidelity metrics report
                </button>
                <button
                  type="button"
                  className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
                  onClick={onRefreshFidelityMetrics}
                >
                  Refresh fidelity from manifest
                </button>
                <button
                  type="button"
                  className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
                  onClick={onExportFidelityMetrics}
                >
                  Export fidelity metrics report
                </button>
              </div>
              <ExperimentFidelityCompareStrip
                manifest={manifest}
                fidelityReport={fidelityReport}
              />
            </>
          )}
          <ExperimentMatrixPanel
            manifest={manifest}
            metricsReport={metricsReport}
            axisRow={matrixAxisRow}
            axisCol={matrixAxisCol}
            onAxisRowChange={onMatrixAxisRowChange}
            onAxisColChange={onMatrixAxisColChange}
          />
          <ExperimentExtendedComparePanel
            filteredRuns={filteredRuns}
            perRunExtended={metricsReport.per_run_extended}
            metricsReport={metricsReport}
            selectedRunIds={extendedCompareRunIds}
            onSelectedRunIdsChange={onExtendedCompareRunIdsChange}
          />
          <ExperimentHandoffEligibilityStrip
            handoff={metricsReport.handoff_eligibility}
            maintainerAckPoseReviewed={maintainerAckPoseReviewed}
            onMaintainerAckPoseReviewedChange={onMaintainerAckPoseReviewedChange}
          />
          <ExperimentImportAdvisoryStrip
            advisoryStatus={workbenchAdvisoryStatus}
            handoff={metricsReport.handoff_eligibility}
          />
        </div>
      ) : null}
    </PanelShell>
  );
}
