import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { ReportDockPresence } from "./reviewPacketPreview";
import {
  exportAnnexBundle,
  getAnnexCache,
  importAnnexBundleDetailed,
} from "./annexReviewStore";
import { exportAnalyticsJson } from "./analyticsDerive";
import { exportFidelityMetricsJson } from "./fidelityMetricsDerive";
import { exportMetricsJson } from "./metricsDerive";
import type { CompareModeId, UnifiedReviewStepId } from "./experimentUnifiedReview";
import {
  safeParseAnalyticsReport,
  safeParseFidelityMetricsReport,
  safeParseMetricsReport,
} from "./experimentImportGuards";
import {
  applyStepWithCompareMode,
  compareModeActivations,
  stepPanelTargets,
} from "./reviewLaneOrchestration";
import {
  loadWorkbenchV2State,
  saveWorkbenchV2State,
  type WorkbenchV2State,
} from "./workbenchV2State";
import type {
  ExperimentAnalyticsReport,
  ExperimentFidelityMetricsReport,
  ExperimentManifest,
  ExperimentMetricsReport,
} from "./experimentSchema";

function runIdToPinnedKey(manifest: ExperimentManifest, runId: string): string | null {
  const idx = manifest.runs.findIndex((r) => r.run_id === runId);
  if (idx < 0) return null;
  return `pinned:${idx}`;
}

export function useExperimentWorkbenchV2(options: {
  manifest: ExperimentManifest;
  analyticsReport: ExperimentAnalyticsReport | null;
  metricsReport: ExperimentMetricsReport | null;
  fidelityReport: ExperimentFidelityMetricsReport | null;
  onAnalyticsActiveChange: (active: boolean) => void;
  onContinuityReviewActiveChange: (active: boolean) => void;
  onCompareModeChange: (active: boolean) => void;
  onF5ActiveChange: (active: boolean) => void;
  setContinuityRunId: (runId: string) => void;
  setAnalyticsOverride: (report: ExperimentAnalyticsReport | null) => void;
  setMetricsOverride: (report: ExperimentMetricsReport | null) => void;
  setFidelityMetricsOverride: (report: ExperimentFidelityMetricsReport | null) => void;
  setCompareA: (key: string | ((prev: string) => string)) => void;
  setCompareB: (key: string | ((prev: string) => string)) => void;
  setExtendedCompareRunIds: (ids: string[] | ((prev: string[]) => string[])) => void;
}) {
  const {
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
  } = options;

  const [v2State, setV2State] = useState<WorkbenchV2State>(() => loadWorkbenchV2State());
  const [dockPacketTabFocus, setDockPacketTabFocus] = useState(0);
  const lastReviewStep = useRef(v2State.review_step);

  useEffect(() => {
    saveWorkbenchV2State(v2State);
  }, [v2State]);

  useEffect(() => {
    if (
      v2State.review_step === "export_packet" &&
      lastReviewStep.current !== "export_packet"
    ) {
      setDockPacketTabFocus((n) => n + 1);
    }
    lastReviewStep.current = v2State.review_step;
  }, [v2State.review_step]);

  const v2ReportPresence = useMemo<ReportDockPresence>(
    () => ({
      f1_analytics: analyticsReport != null && manifest.runs.length > 0,
      f3_annex: Object.keys(getAnnexCache()).length > 0,
      f5_metrics: metricsReport != null,
      f5b_fidelity: fidelityReport != null,
    }),
    [analyticsReport, manifest.runs.length, metricsReport, fidelityReport],
  );

  const dockPreviews = useMemo(
    () => ({
      f1_analytics: analyticsReport ? exportAnalyticsJson(analyticsReport) : undefined,
      f3_annex:
        Object.keys(getAnnexCache()).length > 0 ? exportAnnexBundle(manifest) : undefined,
      f5_metrics: metricsReport ? exportMetricsJson(metricsReport) : undefined,
      f5b_fidelity: fidelityReport ? exportFidelityMetricsJson(fidelityReport) : undefined,
    }),
    [analyticsReport, manifest, metricsReport, fidelityReport],
  );

  const onImportDockSlot = useCallback(
    (slotId: string, text: string): string | null => {
      if (slotId === "f1_analytics") {
        const parsed = safeParseAnalyticsReport(text);
        if (!parsed.ok) return parsed.error;
        setAnalyticsOverride(parsed.data);
        return null;
      }
      if (slotId === "f3_annex") {
        const result = importAnnexBundleDetailed(text);
        if (result.errors.length > 0 && result.imported === 0) {
          return result.errors[0] ?? "annex import failed";
        }
        return null;
      }
      if (slotId === "f5_metrics") {
        const parsed = safeParseMetricsReport(text);
        if (!parsed.ok) return parsed.error;
        setMetricsOverride(parsed.data);
        return null;
      }
      if (slotId === "f5b_fidelity") {
        const parsed = safeParseFidelityMetricsReport(text);
        if (!parsed.ok) return parsed.error;
        setFidelityMetricsOverride(parsed.data);
        return null;
      }
      return `unknown slot: ${slotId}`;
    },
    [setAnalyticsOverride, setFidelityMetricsOverride, setMetricsOverride],
  );

  const onExportDockSlot = useCallback(
    (slotId: string): string | null => {
      if (slotId === "f1_analytics" && analyticsReport) {
        return exportAnalyticsJson(analyticsReport);
      }
      if (slotId === "f3_annex") {
        return exportAnnexBundle(manifest);
      }
      if (slotId === "f5_metrics" && metricsReport) {
        return exportMetricsJson(metricsReport);
      }
      if (slotId === "f5b_fidelity" && fidelityReport) {
        return exportFidelityMetricsJson(fidelityReport);
      }
      return null;
    },
    [analyticsReport, manifest, metricsReport, fidelityReport],
  );

  const activateReviewStep = useCallback(
    (step: UnifiedReviewStepId) => {
      if (step === "compare") {
        const merged = applyStepWithCompareMode(step, v2State.compare_mode);
        onAnalyticsActiveChange(merged.analytics);
        onContinuityReviewActiveChange(merged.continuity);
        onCompareModeChange(merged.compareModeActive);
        onF5ActiveChange(merged.f5Active);
        return;
      }
      const targets = stepPanelTargets(step);
      if (targets.analytics) onAnalyticsActiveChange(true);
      if (targets.continuity) {
        onContinuityReviewActiveChange(true);
        if (v2State.active_run_id) setContinuityRunId(v2State.active_run_id);
      }
      if (targets.f5) onF5ActiveChange(true);
      if (targets.compare) onCompareModeChange(true);
    },
    [
      v2State.compare_mode,
      v2State.active_run_id,
      onAnalyticsActiveChange,
      onContinuityReviewActiveChange,
      onCompareModeChange,
      onF5ActiveChange,
      setContinuityRunId,
    ],
  );

  const applyCompareMode = useCallback(
    (mode: CompareModeId) => {
      const act = compareModeActivations(mode);
      onCompareModeChange(act.compareModeActive);
      onF5ActiveChange(act.f5Active);
    },
    [onCompareModeChange, onF5ActiveChange],
  );

  const syncComparePinned = useCallback(
    (runA: string | null, runB: string | null) => {
      if (runA) {
        const keyA = runIdToPinnedKey(manifest, runA);
        if (keyA) setCompareA(keyA);
      }
      if (runB) {
        const keyB = runIdToPinnedKey(manifest, runB);
        if (keyB) setCompareB(keyB);
      }
      if (runA && runB) {
        setExtendedCompareRunIds([runA, runB].filter(Boolean));
      }
    },
    [manifest, setCompareA, setCompareB, setExtendedCompareRunIds],
  );

  return {
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
  };
}
