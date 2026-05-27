import { useMemo, useState } from "react";
import { PanelShell } from "@/components/GovernanceChrome";
import { BANNER_ANNEX_REVIEW } from "@/governance/banners";
import { CAPTURE_PIPELINE_STEPS } from "@/workflow/captureHandoffCognition";
import { formatImportError } from "./experimentImportGuards";
import {
  exportAnnexBundle,
  getAnnexForRun,
  importAnnexBundleDetailed,
  importAnnexForRun,
} from "./annexReviewStore";
import { deriveExperimentAnalytics } from "./analyticsDerive";
import demoAnnex from "./fixtures/tactical_annex_demo_v1.json";
import type { ExperimentBatchSpec, ExperimentManifest } from "./experimentSchema";
import { TacticalAnnexReviewPanel } from "./TacticalAnnexReviewPanel";

export function ExperimentContinuityReviewPanel({
  manifest,
  batchSpec,
  selectedRunId,
  onSelectRunId,
}: {
  manifest: ExperimentManifest;
  batchSpec?: ExperimentBatchSpec;
  selectedRunId: string;
  onSelectRunId: (runId: string) => void;
}) {
  const [cacheTick, setCacheTick] = useState(0);
  const bumpCache = () => setCacheTick((n) => n + 1);

  const report = useMemo(
    () => deriveExperimentAnalytics(manifest, batchSpec),
    [manifest, batchSpec],
  );

  const perRun = report.per_run.find((r) => r.run_id === selectedRunId);
  const run = manifest.runs.find((r) => r.run_id === selectedRunId);
  const annex = useMemo(
    () => getAnnexForRun(selectedRunId),
    [selectedRunId, cacheTick],
  );

  const importAnnex = () => {
    const text = window.prompt("Paste rt_tactical_capture_annex_v1 JSON");
    if (!text || !selectedRunId) return;
    try {
      importAnnexForRun(selectedRunId, text);
      bumpCache();
    } catch (err) {
      window.alert(
        `Invalid tactical annex: ${formatImportError(err instanceof Error ? err.message : "validation failed")}`,
      );
    }
  };

  const importBundle = () => {
    const text = window.prompt("Paste rt_experiment_annex_bundle_v1 JSON");
    if (!text) return;
    const result = importAnnexBundleDetailed(text);
    bumpCache();
    if (result.errors.length > 0) {
      window.alert(
        `Imported ${result.imported}, skipped ${result.skipped}: ${result.errors.join("; ")}`,
      );
    }
  };

  const loadDemo = () => {
    if (!selectedRunId) return;
    importAnnexForRun(selectedRunId, JSON.stringify(demoAnnex));
    bumpCache();
  };

  const downloadBundle = () => {
    const blob = new Blob([exportAnnexBundle(manifest)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${manifest.experiment_id}-annex-bundle.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  if (manifest.runs.length === 0) {
    return (
      <PanelShell title="Continuity review">
        <p className="text-xs text-slate-500">Pin or import runs to review capture continuity.</p>
      </PanelShell>
    );
  }

  return (
    <PanelShell title="Continuity review">
      <p className="mb-2 text-[10px] text-amber-100/80">{BANNER_ANNEX_REVIEW}</p>
      <label className="mb-3 block text-xs text-slate-400">
        Run
        <select
          className="mt-1 block w-full rounded border border-slate-700 bg-slate-950 px-2 py-1 text-xs"
          value={selectedRunId}
          onChange={(e) => onSelectRunId(e.target.value)}
        >
          {manifest.runs.map((r) => (
            <option key={r.run_id} value={r.run_id}>
              {r.label} ({r.run_id})
            </option>
          ))}
        </select>
      </label>
      <div className="mb-3 flex flex-wrap gap-2">
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
          onClick={importAnnex}
        >
          Import annex JSON
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
          onClick={importBundle}
        >
          Import annex bundle
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
          onClick={loadDemo}
        >
          Load demo annex
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
          onClick={downloadBundle}
        >
          Export annex bundle
        </button>
      </div>

      {perRun && (
        <div className="mb-3 grid gap-2 sm:grid-cols-2 lg:grid-cols-4">
          <div className="rounded border border-slate-800 p-2 text-xs text-slate-400">
            <div className="text-slate-500">mode</div>
            <div className="text-slate-200">{perRun.tactical_mode ?? "—"}</div>
          </div>
          <div className="rounded border border-slate-800 p-2 text-xs text-slate-400">
            <div className="text-slate-500">entities</div>
            <div className="text-slate-200">{perRun.entity_count ?? "—"}</div>
          </div>
          <div className="rounded border border-slate-800 p-2 text-xs text-slate-400">
            <div className="text-slate-500">capture</div>
            <div className="text-slate-200">{perRun.has_capture ? "yes" : "no"}</div>
          </div>
          <div className="rounded border border-slate-800 p-2 text-xs text-slate-400">
            <div className="text-slate-500">normalization</div>
            <div className="font-mono text-[10px] text-slate-300">
              {perRun.normalization_status_ref}
            </div>
          </div>
        </div>
      )}

      {run && (
        <dl className="mb-3 rounded border border-slate-800 bg-slate-950/50 p-2 text-[10px] text-slate-400">
          <dt className="font-medium text-slate-500">capture lineage</dt>
          <dd className="font-mono">id: {run.capture_candidate_id ?? "—"}</dd>
          <dd className="font-mono">staging: {run.capture_staging_ref ?? "—"}</dd>
          {run.tactical_annex_summary && (
            <dd className="mt-1">
              manifest summary — mode switches:{" "}
              {run.tactical_annex_summary.timeline_counts?.mode_switches ?? 0}
            </dd>
          )}
        </dl>
      )}

      <div className="mb-3 flex flex-wrap gap-1">
        {CAPTURE_PIPELINE_STEPS.map((step) => (
          <span
            key={step.id}
            className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-slate-500"
            title={step.note}
          >
            {step.phase}: {step.title}
          </span>
        ))}
      </div>

      <TacticalAnnexReviewPanel
        annex={annex}
        runLabel={run?.label ?? selectedRunId}
        captureStagingRef={run?.capture_staging_ref}
      />
    </PanelShell>
  );
}
