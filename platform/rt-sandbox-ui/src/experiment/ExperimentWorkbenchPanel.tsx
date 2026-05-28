import { useCallback, useEffect, useMemo, useState } from "react";
import { PanelShell } from "@/components/GovernanceChrome";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type { TelemetryChannel } from "@/telemetry/constants";
import { shortSessionId } from "@/workstation/sessionVisualIdentity";
import type { CaptureHandoffRow } from "@/bridge/types";
import { deriveAdvisoryForRow } from "@/handoff/advisoryAggregate";
import type { AdvisoryExperimentRollup } from "@/handoff/advisoryTypes";
import {
  buildExperimentRollupFromWorkbench,
  warnCaptureIdsFromHandoffAndMetrics,
} from "@/handoff/advisoryTriageGrouping";
import { AdvisoryRunBadge } from "@/handoff/AdvisoryRunBadge";
import { ExperimentAnalyticsPanel } from "./ExperimentAnalyticsPanel";
import { ExperimentBatchPanel } from "./ExperimentBatchPanel";
import { ExperimentCompareSection } from "./ExperimentCompareSection";
import { ExperimentF5MetricsSection } from "./ExperimentF5MetricsSection";
import { ExperimentRunSummaryCard } from "./ExperimentRunSummaryCard";
import { ExperimentTrendStrip } from "./ExperimentTrendStrip";
import { SweepCatalogBrowser } from "./SweepCatalogBrowser";
import { deriveExperimentAnalytics } from "./analyticsDerive";
import { ExperimentContinuityReviewPanel } from "./ExperimentContinuityReviewPanel";
import {
  clearAnnexForRun,
  pruneAnnexCacheForManifest,
} from "./annexReviewStore";
import { ExperimentWorkbenchV2Shell } from "./ExperimentWorkbenchV2Shell";
import { useExperimentWorkbenchV2 } from "./useExperimentWorkbenchV2";
import {
  collectFilterOptions,
  collectMatrixAxisKeys,
  EMPTY_F5_FILTERS,
  filterManifestRuns,
  type F5Filters,
} from "./experimentF5UiHelpers";
import {
  safeParseExperimentSpec,
  safeParseFidelityMetricsReport,
  safeParseManifest,
  safeParseMetricsReport,
} from "./experimentImportGuards";
import { ExperimentManifestToolbar } from "./ExperimentManifestToolbar";
import { useJsonPromptImport } from "./useJsonPromptImport";
import {
  compileExperimentSpec,
  compileSpecToYaml,
  type CompileResult,
} from "./experimentSpecCompile";
import {
  createEmptyManifest,
  exportManifestJson,
  loadManifestFromStorage,
  pinRunSnapshot,
  removeRun,
  saveManifestToStorage,
} from "./experimentStore";
import {
  sideFromLive,
  sideFromPinnedRun,
  type CompareSide,
} from "./experimentCompare";
import { deriveExperimentMetrics, exportMetricsJson } from "./metricsDerive";
import {
  deriveExperimentFidelityMetrics,
  exportFidelityMetricsJson,
} from "./fidelityMetricsDerive";
import type {
  ExperimentAnalyticsReport,
  ExperimentBatchSpec,
  ExperimentFidelityMetricsReport,
  ExperimentManifest,
  ExperimentMetricsReport,
  ExperimentSpec,
} from "./experimentSchema";

type SlotRef = {
  sessionId: string;
  label: string;
  snapshots: Partial<Record<TelemetryChannel, ChannelSnapshot>>;
};

export function ExperimentWorkbenchPanel({
  connected,
  slots,
  activeSessionId,
  handoffBySession,
  terrainLayersEnabled,
  compareModeActive,
  onCompareModeChange,
  analyticsActive,
  onAnalyticsActiveChange,
  continuityReviewActive,
  onContinuityReviewActiveChange,
  f5Active,
  onF5ActiveChange,
  onHandoffEligibilityRollupChange,
}: {
  connected: boolean;
  slots: SlotRef[];
  activeSessionId: string | null;
  handoffBySession: Map<string, CaptureHandoffRow[]>;
  terrainLayersEnabled: boolean;
  compareModeActive: boolean;
  onCompareModeChange: (active: boolean) => void;
  analyticsActive: boolean;
  onAnalyticsActiveChange: (active: boolean) => void;
  continuityReviewActive: boolean;
  onContinuityReviewActiveChange: (active: boolean) => void;
  f5Active: boolean;
  onF5ActiveChange: (active: boolean) => void;
  onHandoffEligibilityRollupChange?: (rollup: AdvisoryExperimentRollup | null) => void;
}) {
  const [manifest, setManifest] = useState<ExperimentManifest>(() =>
    loadManifestFromStorage() ?? createEmptyManifest("exp-local"),
  );
  const [compareA, setCompareA] = useState<string>("live:active");
  const [compareB, setCompareB] = useState<string>("pinned:0");
  const [continuityRunId, setContinuityRunId] = useState("");
  const [importedSpec, setImportedSpec] = useState<ExperimentSpec | null>(null);
  const [compiledPreview, setCompiledPreview] = useState<CompileResult | null>(null);
  const [analyticsOverride, setAnalyticsOverride] = useState<ExperimentAnalyticsReport | null>(
    null,
  );
  const [metricsOverride, setMetricsOverride] = useState<ExperimentMetricsReport | null>(
    null,
  );
  const [fidelityMetricsOverride, setFidelityMetricsOverride] =
    useState<ExperimentFidelityMetricsReport | null>(null);
  const [f5Filters, setF5Filters] = useState<F5Filters>(EMPTY_F5_FILTERS);
  const [extendedCompareRunIds, setExtendedCompareRunIds] = useState<string[]>([]);
  const [matrixAxisRow, setMatrixAxisRow] = useState("");
  const [matrixAxisCol, setMatrixAxisCol] = useState("");
  const [maintainerAckPoseReviewed, setMaintainerAckPoseReviewed] = useState(false);
  const [batchSpec, setBatchSpec] = useState<ExperimentBatchSpec>({
    schema: "rt_experiment_batch_v1",
    experiment_id: manifest.experiment_id,
    default_dwell_s: 2,
    runs: [
      { run_id: "run-a", label: "run A", dwell_s: 2 },
      { run_id: "run-b", label: "run B", dwell_s: 2 },
    ],
  });

  useEffect(() => {
    saveManifestToStorage(manifest);
  }, [manifest]);

  useEffect(() => {
    if (manifest.runs.length === 0) {
      setContinuityRunId("");
      return;
    }
    if (!manifest.runs.some((r) => r.run_id === continuityRunId)) {
      setContinuityRunId(manifest.runs[0].run_id);
    }
  }, [manifest.runs, continuityRunId]);

  const analyticsReport = useMemo(() => {
    if (analyticsOverride) return analyticsOverride;
    return deriveExperimentAnalytics(manifest, batchSpec);
  }, [analyticsOverride, manifest, batchSpec]);

  const metricsReport = useMemo(() => {
    if (metricsOverride) return metricsOverride;
    if (manifest.runs.length === 0) return null;
    return deriveExperimentMetrics(manifest, analyticsReport, {
      batchSpec,
      spec: importedSpec ?? undefined,
      maintainerAckPoseReviewed,
    });
  }, [
    metricsOverride,
    manifest,
    analyticsReport,
    batchSpec,
    importedSpec,
    maintainerAckPoseReviewed,
  ]);

  const fidelityReport = useMemo(() => {
    if (fidelityMetricsOverride) return fidelityMetricsOverride;
    if (manifest.runs.length === 0) return null;
    return deriveExperimentFidelityMetrics(manifest, metricsReport ?? undefined, {
      spec: importedSpec ?? undefined,
    });
  }, [fidelityMetricsOverride, manifest, metricsReport, importedSpec]);

  const {
    v2State,
    setV2State,
    v2ReportPresence,
    dockPreviews,
    onImportDockSlot,
    onExportDockSlot,
    activateReviewStep,
    applyCompareMode,
    syncComparePinned,
    dockPacketTabFocus,
  } = useExperimentWorkbenchV2({
    manifest,
    analyticsReport,
    metricsReport,
    fidelityReport,
    onAnalyticsActiveChange,
    onContinuityReviewActiveChange,
    onCompareModeChange,
    onF5ActiveChange,
    setContinuityRunId,
    setAnalyticsOverride,
    setMetricsOverride,
    setFidelityMetricsOverride,
    setCompareA,
    setCompareB,
    setExtendedCompareRunIds,
  });

  useEffect(() => {
    if (!onHandoffEligibilityRollupChange) return;
    if (!metricsReport || !activeSessionId) {
      onHandoffEligibilityRollupChange(null);
      return;
    }
    const rows = handoffBySession.get(activeSessionId) ?? [];
    const hints = new Map<string, string | undefined>();
    for (const run of manifest.runs) {
      const cid = run.capture_candidate_id;
      if (!cid) continue;
      const ext = metricsReport.per_run_extended.find((e) => e.run_id === run.run_id);
      hints.set(cid, ext?.handoff_eligibility_hint);
    }
    const level = metricsReport.handoff_eligibility.experiment_level;
    const warnIds = warnCaptureIdsFromHandoffAndMetrics(rows, null, level, hints);
    onHandoffEligibilityRollupChange(
      buildExperimentRollupFromWorkbench(level, warnIds),
    );
  }, [
    metricsReport,
    activeSessionId,
    handoffBySession,
    manifest.runs,
    onHandoffEligibilityRollupChange,
  ]);

  const fidelityCouplingPresent = useMemo(() => {
    if (fidelityReport?.coupling_required) return true;
    return manifest.runs.some((r) => r.fidelity_context?.enable_fidelity_coupling);
  }, [fidelityReport, manifest.runs]);

  const filterOptions = useMemo(() => {
    if (!metricsReport) {
      return {
        experiment_class: [],
        tactical_mode: [],
        terrain_preset: [],
        visibility_context: [],
      };
    }
    return collectFilterOptions(manifest.runs, metricsReport.per_run_extended);
  }, [manifest.runs, metricsReport]);

  const filteredRuns = useMemo(() => {
    if (!metricsReport) return manifest.runs;
    return filterManifestRuns(
      manifest.runs,
      f5Filters,
      metricsReport.per_run_extended,
    );
  }, [manifest.runs, f5Filters, metricsReport]);

  const advisoryByCaptureId = useMemo(() => {
    const rows = activeSessionId ? (handoffBySession.get(activeSessionId) ?? []) : [];
    const map = new Map<string, ReturnType<typeof deriveAdvisoryForRow>>();
    for (const row of rows) {
      map.set(
        row.capture_candidate_id,
        deriveAdvisoryForRow(row, undefined, {
          poseAttested: maintainerAckPoseReviewed,
        }),
      );
    }
    return map;
  }, [activeSessionId, handoffBySession, maintainerAckPoseReviewed]);

  const workbenchAdvisoryStatus = useMemo(() => {
    const runWithCapture = manifest.runs.find((r) => r.capture_candidate_id);
    if (!runWithCapture?.capture_candidate_id) return null;
    return advisoryByCaptureId.get(runWithCapture.capture_candidate_id) ?? null;
  }, [manifest.runs, advisoryByCaptureId]);

  useEffect(() => {
    const keys = collectMatrixAxisKeys(manifest.runs);
    if (keys.length >= 2) {
      if (!keys.includes(matrixAxisRow)) setMatrixAxisRow(keys[0]);
      if (!keys.includes(matrixAxisCol)) setMatrixAxisCol(keys[1] ?? keys[0]);
    }
  }, [manifest.runs, matrixAxisRow, matrixAxisCol]);

  useEffect(() => {
    setExtendedCompareRunIds((ids) =>
      ids.filter((id) => filteredRuns.some((r) => r.run_id === id)),
    );
  }, [filteredRuns]);

  const resolveSide = useCallback(
    (key: string): CompareSide | null => {
      if (key.startsWith("live:")) {
        const sid = key.slice(5);
        const slot =
          sid === "active"
            ? slots.find((s) => s.sessionId === activeSessionId)
            : slots.find((s) => s.sessionId === sid);
        if (!slot) return null;
        const tactical = slot.snapshots.tactical_state?.payload ?? {};
        return sideFromLive(
          slot.label,
          slot.sessionId,
          {
            tactical_state: tactical,
            world_summary: slot.snapshots.world_summary?.payload,
            lifecycle_state: slot.snapshots.lifecycle_state?.payload,
          },
          terrainLayersEnabled ? "fictional terrain layers on" : undefined,
        );
      }
      if (key.startsWith("pinned:")) {
        const idx = Number(key.slice(7));
        const run = manifest.runs[idx];
        if (!run) return null;
        return sideFromPinnedRun(run);
      }
      return null;
    },
    [slots, activeSessionId, manifest.runs, terrainLayersEnabled],
  );

  const sideA = useMemo(() => resolveSide(compareA), [compareA, resolveSide]);
  const sideB = useMemo(() => resolveSide(compareB), [compareB, resolveSide]);

  const promptJsonImport = useJsonPromptImport();

  const pinActive = () => {
    if (!activeSessionId) return;
    const slot = slots.find((s) => s.sessionId === activeSessionId);
    if (!slot) return;
    const runId = `pin-${Date.now()}`;
    setManifest((m) =>
      pinRunSnapshot({
        manifest: m,
        runId,
        label: `pinned ${shortSessionId(activeSessionId)}`,
        sessionId: activeSessionId,
        snapshots: slot.snapshots,
        terrainLayersEnabled,
      }),
    );
  };

  const slotSessionIds = useMemo(
    () => new Set(slots.map((s) => s.sessionId)),
    [slots],
  );

  const normalizeCompareKey = useCallback(
    (key: string, m: ExperimentManifest): string => {
      if (key.startsWith("pinned:")) {
        const idx = Number(key.slice(7));
        if (!Number.isFinite(idx) || idx < 0 || idx >= m.runs.length) {
          return m.runs.length > 0 ? "pinned:0" : "pinned:0";
        }
        return key;
      }
      if (key.startsWith("live:")) {
        const sid = key.slice(5);
        if (sid === "active") return key;
        if (!slotSessionIds.has(sid)) return "live:active";
        return key;
      }
      return "live:active";
    },
    [slotSessionIds],
  );

  useEffect(() => {
    setCompareA((a) => normalizeCompareKey(a, manifest));
    setCompareB((b) => normalizeCompareKey(b, manifest));
  }, [manifest.runs, normalizeCompareKey, manifest]);

  const importManifest = () => {
    promptJsonImport({
      promptMessage: "Paste rt_experiment_manifest_v1 JSON",
      invalidLabel: "manifest",
      parse: safeParseManifest,
      onSuccess: (data) => {
        setManifest(data);
        setMetricsOverride(null);
        pruneAnnexCacheForManifest(data);
        setBatchSpec((spec) => ({ ...spec, experiment_id: data.experiment_id }));
        setCompareA((a) => normalizeCompareKey(a, data));
        setCompareB((b) => normalizeCompareKey(b, data));
      },
    });
  };

  const importSpec = () => {
    promptJsonImport({
      promptMessage: "Paste rt_experiment_spec_v1 JSON",
      invalidLabel: "spec",
      parse: safeParseExperimentSpec,
      onSuccess: (data) => {
        try {
          const compiled = compileExperimentSpec(data);
          setImportedSpec(data);
          setCompiledPreview(compiled);
        } catch (err) {
          window.alert(err instanceof Error ? err.message : String(err));
        }
      },
    });
  };

  const applyCompiledBatch = () => {
    if (!compiledPreview) return;
    setBatchSpec(compiledPreview.batch);
    setManifest((m) => ({
      ...m,
      experiment_id: compiledPreview.batch.experiment_id,
    }));
  };

  const importMetrics = () => {
    promptJsonImport({
      promptMessage: "Paste rt_experiment_metrics_report_v1 JSON",
      invalidLabel: "metrics report",
      parse: safeParseMetricsReport,
      onSuccess: (data) => setMetricsOverride(data),
    });
  };

  const importFidelityMetrics = () => {
    promptJsonImport({
      promptMessage: "Paste rt_experiment_fidelity_metrics_report_v1 JSON",
      invalidLabel: "fidelity metrics report",
      parse: safeParseFidelityMetricsReport,
      onSuccess: (data) => setFidelityMetricsOverride(data),
    });
  };

  const exportManifest = () => {
    const blob = new Blob([exportManifestJson(manifest)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${manifest.experiment_id}-manifest.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const exportMetrics = () => {
    if (!metricsReport) return;
    const blob = new Blob(
      [exportMetricsJson(metricsReport, { derived_at_utc: new Date().toISOString() })],
      { type: "application/json" },
    );
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${manifest.experiment_id}-metrics_report.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const exportFidelityMetrics = () => {
    if (!fidelityReport) return;
    const blob = new Blob([exportFidelityMetricsJson(fidelityReport)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${manifest.experiment_id}-fidelity_metrics_report.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const compareOptions = useMemo(() => {
    const opts: { value: string; label: string }[] = [];
    for (const slot of slots) {
      opts.push({
        value: `live:${slot.sessionId}`,
        label: `live · ${shortSessionId(slot.sessionId)}`,
      });
    }
    manifest.runs.forEach((r, i) => {
      opts.push({ value: `pinned:${i}`, label: `pinned · ${r.label}` });
    });
    return opts;
  }, [slots, manifest.runs]);

  const specCompileCliHint = importedSpec
    ? `python3 scripts/rt/rt_experiment_spec_compile.py --spec fixtures/rt_experiments/f5_spec_examples/<spec>.json --out runs/rt_sandbox/experiments/${importedSpec.experiment_id}/batch.yaml`
    : null;

  const metricsCliHint = `python3 scripts/rt/rt_experiment_metrics.py --manifest <path> [--batch <path>] [--spec <path>] --out metrics_report.json`;

  const fidelityMetricsCliHint =
    "python3 scripts/rt/rt_experiment_fidelity_metrics.py --manifest <path> [--metrics <path>] [--spec <path>] --repo-root . --out fidelity_metrics_report.json";

  const compiledYamlPreview =
    importedSpec && compiledPreview ? compileSpecToYaml(importedSpec) : null;

  return (
    <div className="space-y-4" data-testid="experiment-workbench">
      <ExperimentWorkbenchV2Shell
        v2State={v2State}
        onV2StateChange={setV2State}
        manifest={manifest}
        reportPresence={v2ReportPresence}
        dockPreviews={dockPreviews}
        onImportDockSlot={onImportDockSlot}
        onExportDockSlot={onExportDockSlot}
        onActivateStep={activateReviewStep}
        onContinuityRunId={setContinuityRunId}
        onSyncComparePinned={syncComparePinned}
        onApplyCompareMode={applyCompareMode}
        dockPacketTabFocus={dockPacketTabFocus}
      />
      <PanelShell title="Experiment workbench">
        <ExperimentManifestToolbar
          experimentId={manifest.experiment_id}
          onExperimentIdChange={(value) =>
            setManifest((m) => ({ ...m, experiment_id: value }))
          }
          connected={connected}
          activeSessionId={activeSessionId}
          onPinActive={pinActive}
          onImportManifest={importManifest}
          onExportManifest={exportManifest}
          onImportSpec={importSpec}
          compareModeActive={compareModeActive}
          onCompareModeChange={onCompareModeChange}
          analyticsActive={analyticsActive}
          onAnalyticsActiveChange={onAnalyticsActiveChange}
          continuityReviewActive={continuityReviewActive}
          onContinuityReviewActiveChange={onContinuityReviewActiveChange}
          f5Active={f5Active}
          onF5ActiveChange={onF5ActiveChange}
        />

        {compiledYamlPreview && (
          <div className="mb-3 space-y-2 rounded border border-slate-800 bg-slate-950/60 p-2">
            <p className="text-[10px] text-slate-500">Compiled batch preview (read-only)</p>
            <pre className="max-h-32 overflow-auto text-[10px] text-slate-400">
              {compiledYamlPreview.slice(0, 1200)}
              {compiledYamlPreview.length > 1200 ? "\n…" : ""}
            </pre>
            {specCompileCliHint && (
              <p className="font-mono text-[10px] text-slate-500">{specCompileCliHint}</p>
            )}
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-800 px-2 py-0.5 text-xs text-slate-200"
              onClick={applyCompiledBatch}
            >
              Apply compiled batch
            </button>
          </div>
        )}

        <div className="mb-3 grid gap-2 sm:grid-cols-2 lg:grid-cols-3">
          {filteredRuns.map((run) => (
            <div key={run.run_id} className="relative">
              <ExperimentRunSummaryCard run={run} />
              {f5Active && run.capture_candidate_id && (
                <AdvisoryRunBadge
                  status={advisoryByCaptureId.get(run.capture_candidate_id) ?? null}
                />
              )}
              <button
                type="button"
                className="absolute right-1 top-1 text-[10px] text-red-400"
                onClick={() => {
                  clearAnnexForRun(run.run_id);
                  setManifest((m) => {
                    const next = removeRun(m, run.run_id);
                    pruneAnnexCacheForManifest(next);
                    return next;
                  });
                }}
              >
                remove
              </button>
            </div>
          ))}
        </div>
        {compareModeActive && (
          <ExperimentCompareSection
            compareA={compareA}
            compareB={compareB}
            onCompareAChange={setCompareA}
            onCompareBChange={setCompareB}
            compareOptions={compareOptions}
            sideA={sideA}
            sideB={sideB}
            experimentId={manifest.experiment_id}
          />
        )}
      </PanelShell>
      <SweepCatalogBrowser
        experimentId={manifest.experiment_id}
        onApplyBatchSpec={(spec) => {
          setBatchSpec(spec);
          setManifest((m) => ({ ...m, experiment_id: spec.experiment_id }));
        }}
      />
      {analyticsActive && (
        <>
          <ExperimentAnalyticsPanel manifest={manifest} batchSpec={batchSpec} />
          {manifest.runs.length > 0 &&
            (metricsReport?.experiment_class ??
              manifest.runs.find((r) => r.experiment_class)?.experiment_class ??
              importedSpec?.experiment_class) !== "repeatability_sweep" && (
              <ExperimentTrendStrip perRun={analyticsReport.per_run} />
            )}
        </>
      )}
      {continuityReviewActive && (
        <ExperimentContinuityReviewPanel
          manifest={manifest}
          batchSpec={batchSpec}
          selectedRunId={continuityRunId || manifest.runs[0]?.run_id || ""}
          onSelectRunId={setContinuityRunId}
        />
      )}
      {f5Active && (
        <ExperimentF5MetricsSection
          manifest={manifest}
          metricsReport={metricsReport}
          fidelityReport={fidelityReport}
          analyticsReport={analyticsReport}
          filteredRuns={filteredRuns}
          f5Filters={f5Filters}
          onF5FiltersChange={setF5Filters}
          filterOptions={filterOptions}
          fidelityCouplingPresent={fidelityCouplingPresent}
          matrixAxisRow={matrixAxisRow}
          matrixAxisCol={matrixAxisCol}
          onMatrixAxisRowChange={setMatrixAxisRow}
          onMatrixAxisColChange={setMatrixAxisCol}
          extendedCompareRunIds={extendedCompareRunIds}
          onExtendedCompareRunIdsChange={setExtendedCompareRunIds}
          maintainerAckPoseReviewed={maintainerAckPoseReviewed}
          onMaintainerAckPoseReviewedChange={setMaintainerAckPoseReviewed}
          workbenchAdvisoryStatus={workbenchAdvisoryStatus}
          metricsCliHint={metricsCliHint}
          fidelityMetricsCliHint={fidelityMetricsCliHint}
          onImportMetrics={importMetrics}
          onExportMetrics={exportMetrics}
          onImportFidelityMetrics={importFidelityMetrics}
          onExportFidelityMetrics={exportFidelityMetrics}
          onRefreshMetrics={() => setMetricsOverride(null)}
          onRefreshFidelityMetrics={() => setFidelityMetricsOverride(null)}
        />
      )}
      <ExperimentBatchPanel
        experimentId={manifest.experiment_id}
        batchSpec={batchSpec}
        onBatchSpecChange={setBatchSpec}
      />
    </div>
  );
}
