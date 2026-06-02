import { useEffect, useMemo, useState } from "react";
import { FlaskConical } from "lucide-react";
import type { UiEntity } from "@/editing/localEntityMirror";
import { PanelShell } from "./GovernanceChrome";
import {
  entitiesToRtLayoutScenario,
  layoutIdForSession,
} from "@/layout/rtLayoutFromEntities";
import {
  buildMcJobPreview,
  formatEstimatedDuration,
  estimateMcJobDurationSec,
  type McJobPreview,
} from "@/layout/mcJobPreview";
import {
  geometryFingerprint,
  translateLayoutToProfile,
  validateLayout,
  type McProfilePreview,
} from "@/layout/rtLayoutMcProfile";
import {
  copyLayoutMcHandoffBundle,
  copyMcJobPreview,
  copyRtLayoutScenario,
  downloadLayoutMcHandoffBundle,
  downloadMcJobPreview,
  downloadRtLayoutScenario,
  isPreparedJobStale,
} from "@/layout/layoutMcHandoff";
import {
  presetById,
  SCENARIO_EVALUATION_PRESETS,
  type ScenarioEvaluationPresetId,
} from "@/layout/scenarioEvaluationPresets";

export type LayoutStatus = "empty" | "invalid" | "ready";

export type TranslatorStatus = "idle" | "ready" | "preview" | "error";

function layoutStatusLabel(status: LayoutStatus): string {
  switch (status) {
    case "empty":
      return "Empty";
    case "invalid":
      return "Invalid";
    case "ready":
      return "Ready";
  }
}

function translatorStatusLabel(status: TranslatorStatus): string {
  switch (status) {
    case "idle":
      return "Idle";
    case "ready":
      return "Ready";
    case "preview":
      return "Preview";
    case "error":
      return "Error";
  }
}

function deriveLayoutStatus(
  entities: UiEntity[],
  validationOk: boolean,
): LayoutStatus {
  if (entities.length === 0) return "empty";
  if (!validationOk) return "invalid";
  return "ready";
}

export function ScenarioEvaluationPanel({
  entities,
  sessionId,
  disabled = false,
}: {
  entities: UiEntity[];
  sessionId: string | null;
  disabled?: boolean;
}) {
  const [presetId, setPresetId] =
    useState<ScenarioEvaluationPresetId>("standard");
  const [preview, setPreview] = useState<McProfilePreview | null>(null);
  const [previewError, setPreviewError] = useState<string | null>(null);
  const [previewOpen, setPreviewOpen] = useState(false);
  const [preparedJob, setPreparedJob] = useState<McJobPreview | null>(null);
  const [preparedOpen, setPreparedOpen] = useState(false);
  const [handoffMessage, setHandoffMessage] = useState<string | null>(null);

  const preset = presetById(presetId);

  const pendingJobEstimate = useMemo(() => {
    if (!preview) return null;
    const sec = estimateMcJobDurationSec(preset.runs);
    return formatEstimatedDuration(sec);
  }, [preview, preset.runs]);

  const layout = useMemo(
    () =>
      entitiesToRtLayoutScenario(entities, {
        layoutId: layoutIdForSession(sessionId),
      }),
    [entities, sessionId],
  );

  const validation = useMemo(
    () => validateLayout(layout as unknown as Record<string, unknown>),
    [layout],
  );

  const layoutStatus = deriveLayoutStatus(entities, validation.ok);

  const geometryId = useMemo(() => {
    if (!validation.ok) return null;
    try {
      return geometryFingerprint(layout);
    } catch {
      return null;
    }
  }, [layout, validation.ok]);

  const translatorStatus: TranslatorStatus = previewError
    ? "error"
    : preview
      ? "preview"
      : layoutStatus === "ready"
        ? "ready"
        : "idle";

  const warningCount = preview
    ? preview.warnings.length
    : validation.warnings.length;

  const preparedJobStale = isPreparedJobStale(preparedJob, geometryId);

  useEffect(() => {
    setPreparedJob(null);
    setPreparedOpen(false);
    setHandoffMessage(null);
  }, [presetId, sessionId]);

  const runHandoff = async (
    label: string,
    action: () => void | Promise<{ ok: boolean; error?: string }>,
  ) => {
    setHandoffMessage(null);
    try {
      const result = action();
      if (result instanceof Promise) {
        const awaited = await result;
        if (!awaited.ok) {
          setHandoffMessage(`${label} failed: ${awaited.error ?? "unknown error"}`);
          return;
        }
      }
      setHandoffMessage(`${label} ready (browser only — not executed).`);
    } catch (err) {
      setHandoffMessage(
        `${label} failed: ${err instanceof Error ? err.message : String(err)}`,
      );
    }
  };

  const onGenerateProfile = () => {
    setPreviewError(null);
    setPreparedJob(null);
    setPreparedOpen(false);
    try {
      const next = translateLayoutToProfile(layout);
      setPreview(next);
      setPreviewOpen(true);
    } catch (err) {
      setPreview(null);
      setPreviewError(err instanceof Error ? err.message : String(err));
      setPreviewOpen(true);
    }
  };

  const onPrepareMcJob = () => {
    if (!preview) return;
    const job = buildMcJobPreview(preview, preset);
    setPreparedJob(job);
    setPreparedOpen(true);
  };

  return (
    <PanelShell title="Scenario Evaluation" icon={FlaskConical} variant="tertiary">
      <p className="mb-3 text-xs text-slate-500">
        Read-only layout evaluation and freeze-ready browser handoff. Does not run
        Monte Carlo or invoke the RT bridge.
      </p>

      <dl className="mb-3 grid grid-cols-2 gap-x-3 gap-y-2 text-xs">
        <div>
          <dt className="text-slate-500">Layout status</dt>
          <dd className="font-medium text-slate-200">
            {layoutStatusLabel(layoutStatus)}
          </dd>
        </div>
        <div>
          <dt className="text-slate-500">Geometry ID</dt>
          <dd className="break-all font-mono text-[10px] text-cyan-200/90">
            {geometryId ?? "—"}
          </dd>
        </div>
        <div>
          <dt className="text-slate-500">Translator status</dt>
          <dd className="font-medium text-slate-200">
            {translatorStatusLabel(translatorStatus)}
          </dd>
        </div>
        <div>
          <dt className="text-slate-500">Warnings</dt>
          <dd className="font-medium text-slate-200">{warningCount}</dd>
        </div>
      </dl>

      {validation.issues.length > 0 && (
        <ul className="mb-3 list-inside list-disc text-xs text-amber-200/90">
          {validation.issues.map((issue) => (
            <li key={issue}>{issue}</li>
          ))}
        </ul>
      )}

      <fieldset className="mb-3" disabled={disabled}>
        <legend className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
          Evaluation presets
        </legend>
        <div className="flex flex-wrap gap-2">
          {SCENARIO_EVALUATION_PRESETS.map((row) => (
            <label
              key={row.id}
              className="flex cursor-pointer items-center gap-2 rounded border border-slate-700/80 bg-slate-950/40 px-2 py-1.5 text-xs text-slate-300 has-[:checked]:border-cyan-600/60 has-[:checked]:bg-cyan-950/30"
            >
              <input
                type="radio"
                name="mc-eval-preset"
                value={row.id}
                checked={presetId === row.id}
                onChange={() => setPresetId(row.id)}
                className="accent-cyan-500"
              />
              <span>
                {row.label}{" "}
                <span className="text-slate-500">({row.runs} runs)</span>
              </span>
            </label>
          ))}
        </div>
        <p className="mt-2 text-[10px] text-slate-500">{preset.description}</p>
      </fieldset>

      <button
        type="button"
        disabled={disabled || layoutStatus !== "ready"}
        onClick={onGenerateProfile}
        className="mb-3 w-full rounded border border-cyan-700/50 bg-cyan-950/40 px-3 py-2 text-xs font-semibold uppercase tracking-wide text-cyan-100 enabled:hover:bg-cyan-900/50 disabled:cursor-not-allowed disabled:opacity-40"
      >
        Generate MC Profile
      </button>

      <details
        open={previewOpen}
        onToggle={(event) => setPreviewOpen(event.currentTarget.open)}
        className="rounded border border-slate-800/80 bg-slate-950/30"
      >
        <summary className="cursor-pointer px-3 py-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
          MC Preview
        </summary>
        <div className="space-y-3 border-t border-slate-800/80 px-3 py-3 text-xs text-slate-300">
          {previewError && (
            <p className="text-amber-200/90" role="alert">
              {previewError}
            </p>
          )}
          {!preview && !previewError && (
            <p className="text-slate-500">
              Generate a profile preview to see scenario mapping and launch args.
            </p>
          )}
          {preview && (
            <>
              <div>
                <p className="text-slate-500">geometry_id</p>
                <p className="break-all font-mono text-[10px] text-cyan-200/90">
                  {preview.geometry_id}
                </p>
              </div>
              <div>
                <p className="text-slate-500">scenario</p>
                <p className="font-mono text-slate-200">{preview.scenario_suggestion}</p>
              </div>
              <div>
                <p className="text-slate-500">launch args</p>
                <p className="break-all font-mono text-[10px] text-slate-200">
                  {preview.launch_args || "—"}
                </p>
              </div>
              {preview.unsupported_fields.length > 0 && (
                <div>
                  <p className="mb-1 text-slate-500">unsupported entities</p>
                  <ul className="list-inside list-disc text-slate-400">
                    {preview.unsupported_fields.map((row) => (
                      <li key={`${row.field}-${row.reason}`}>
                        <span className="font-mono text-slate-300">{row.field}</span>
                        {": "}
                        {row.reason}
                      </li>
                    ))}
                  </ul>
                </div>
              )}
              {preview.warnings.length > 0 && (
                <div>
                  <p className="mb-1 text-slate-500">warnings</p>
                  <ul className="list-inside list-disc text-amber-200/80">
                    {preview.warnings.map((warning) => (
                      <li key={warning}>{warning}</li>
                    ))}
                  </ul>
                </div>
              )}
              {pendingJobEstimate && (
                <p className="text-[10px] text-slate-500">
                  Job sizing for {preset.label}: {preset.runs} runs, estimated{" "}
                  {pendingJobEstimate} (heuristic, not scheduled).
                </p>
              )}
            </>
          )}
        </div>
      </details>

      <button
        type="button"
        disabled={disabled || !preview || Boolean(previewError)}
        onClick={onPrepareMcJob}
        className="mb-3 mt-3 w-full rounded border border-violet-700/50 bg-violet-950/35 px-3 py-2 text-xs font-semibold uppercase tracking-wide text-violet-100 enabled:hover:bg-violet-900/45 disabled:cursor-not-allowed disabled:opacity-40"
      >
        Prepare MC Job
      </button>

      <details
        open={preparedOpen}
        onToggle={(event) => setPreparedOpen(event.currentTarget.open)}
        className="rounded border border-violet-900/50 bg-violet-950/20"
      >
        <summary className="cursor-pointer px-3 py-2 text-xs font-semibold uppercase tracking-wide text-violet-300/90">
          Prepared MC Job
        </summary>
        <div className="space-y-3 border-t border-violet-900/40 px-3 py-3 text-xs text-slate-300">
          {!preparedJob && (
            <p className="text-slate-500">
              Generate a profile preview, then prepare a read-only job summary for
              offline handoff. Nothing runs from this panel.
            </p>
          )}
          {preparedJobStale && preparedJob && (
            <div
              role="alert"
              className="rounded border border-amber-700/60 bg-amber-950/40 p-3 text-amber-100/95"
            >
              <p className="font-semibold uppercase tracking-wide text-amber-200/90">
                Stale prepared job
              </p>
              <p className="mt-1">
                Current layout geometry differs from the prepared job fingerprint.
                Re-prepare after edits before offline Monte Carlo handoff.
              </p>
              <p className="mt-2 font-mono text-[10px] text-amber-200/80">
                current: {geometryId ?? "—"}
              </p>
              <p className="font-mono text-[10px] text-amber-200/80">
                prepared: {preparedJob.geometry_id}
              </p>
            </div>
          )}
          {preparedJob && (
            <dl className="grid grid-cols-1 gap-3 sm:grid-cols-2">
              <div className="sm:col-span-2">
                <dt className="text-slate-500">geometry_id</dt>
                <dd className="break-all font-mono text-[10px] text-cyan-200/90">
                  {preparedJob.geometry_id}
                </dd>
              </div>
              <div>
                <dt className="text-slate-500">preset</dt>
                <dd className="text-slate-200">
                  {preparedJob.preset_label}{" "}
                  <span className="text-slate-500">({preparedJob.preset})</span>
                </dd>
              </div>
              <div>
                <dt className="text-slate-500">run_count</dt>
                <dd className="font-mono text-slate-200">{preparedJob.run_count}</dd>
              </div>
              <div>
                <dt className="text-slate-500">estimated_duration</dt>
                <dd className="text-slate-200">
                  {preparedJob.estimated_duration}
                  <span className="ml-1 text-slate-500">
                    ({preparedJob.estimated_duration_sec}s)
                  </span>
                </dd>
              </div>
              <div>
                <dt className="text-slate-500">scenario_label</dt>
                <dd className="font-mono text-slate-200">{preparedJob.scenario_label}</dd>
              </div>
              <div className="sm:col-span-2">
                <dt className="text-slate-500">launch_args</dt>
                <dd className="break-all font-mono text-[10px] text-slate-200">
                  {preparedJob.launch_args || "—"}
                </dd>
              </div>
              <div className="sm:col-span-2">
                <dt className="text-slate-500">warnings</dt>
                <dd>
                  {preparedJob.warnings.length === 0 ? (
                    <span className="text-slate-500">None</span>
                  ) : (
                    <ul className="list-inside list-disc text-amber-200/80">
                      {preparedJob.warnings.map((warning) => (
                        <li key={warning}>{warning}</li>
                      ))}
                    </ul>
                  )}
                </dd>
              </div>
              <div className="sm:col-span-2 text-[10px] text-slate-600">
                Prepared {preparedJob.prepared_utc} · layout{" "}
                <span className="font-mono">{preparedJob.source_layout_id}</span>
              </div>
            </dl>
          )}
          {(validation.ok || preparedJob) && (
            <div className="space-y-2 border-t border-violet-900/40 pt-3">
              <p className="text-xs font-semibold uppercase tracking-wide text-slate-400">
                Freeze-ready handoff
              </p>
              <p className="text-[10px] text-slate-500">
                Exports <span className="font-mono">rt_layout_scenario_v1</span> and{" "}
                <span className="font-mono">rt_mc_job_preview_v1</span> JSON via browser
                clipboard or download only.
              </p>
              <div className="flex flex-wrap gap-2">
                <button
                  type="button"
                  disabled={disabled || !validation.ok}
                  onClick={() =>
                    void runHandoff("Layout JSON copied", () => copyRtLayoutScenario(layout))
                  }
                  className="rounded border border-slate-600/80 px-2 py-1 text-[10px] font-semibold uppercase text-slate-200 enabled:hover:bg-slate-800/60 disabled:opacity-40"
                >
                  Copy layout JSON
                </button>
                <button
                  type="button"
                  disabled={disabled || !validation.ok}
                  onClick={() =>
                    void runHandoff("Layout JSON downloaded", () => {
                      downloadRtLayoutScenario(layout);
                    })
                  }
                  className="rounded border border-slate-600/80 px-2 py-1 text-[10px] font-semibold uppercase text-slate-200 enabled:hover:bg-slate-800/60 disabled:opacity-40"
                >
                  Download layout
                </button>
                <button
                  type="button"
                  disabled={disabled || !preparedJob}
                  onClick={() =>
                    preparedJob &&
                    void runHandoff("Job JSON copied", () => copyMcJobPreview(preparedJob))
                  }
                  className="rounded border border-slate-600/80 px-2 py-1 text-[10px] font-semibold uppercase text-slate-200 enabled:hover:bg-slate-800/60 disabled:opacity-40"
                >
                  Copy job JSON
                </button>
                <button
                  type="button"
                  disabled={disabled || !preparedJob}
                  onClick={() =>
                    preparedJob &&
                    void runHandoff("Job JSON downloaded", () => {
                      downloadMcJobPreview(preparedJob);
                    })
                  }
                  className="rounded border border-slate-600/80 px-2 py-1 text-[10px] font-semibold uppercase text-slate-200 enabled:hover:bg-slate-800/60 disabled:opacity-40"
                >
                  Download job
                </button>
                <button
                  type="button"
                  disabled={disabled || !preparedJob || !validation.ok}
                  onClick={() =>
                    preparedJob &&
                    void runHandoff("Handoff bundle copied", () =>
                      copyLayoutMcHandoffBundle(layout, preparedJob),
                    )
                  }
                  className="rounded border border-cyan-700/50 px-2 py-1 text-[10px] font-semibold uppercase text-cyan-100 enabled:hover:bg-cyan-950/50 disabled:opacity-40"
                >
                  Copy bundle
                </button>
                <button
                  type="button"
                  disabled={disabled || !preparedJob || !validation.ok}
                  onClick={() =>
                    preparedJob &&
                    void runHandoff("Handoff bundle downloaded", () => {
                      downloadLayoutMcHandoffBundle(layout, preparedJob);
                    })
                  }
                  className="rounded border border-cyan-700/50 px-2 py-1 text-[10px] font-semibold uppercase text-cyan-100 enabled:hover:bg-cyan-950/50 disabled:opacity-40"
                >
                  Download bundle
                </button>
              </div>
              {preparedJobStale && (
                <p className="text-[10px] text-amber-200/80">
                  Job exports reflect the prepared snapshot; layout exports use the live
                  editor. Re-prepare to align geometry_id.
                </p>
              )}
              {handoffMessage && (
                <p className="text-[10px] text-slate-400" role="status">
                  {handoffMessage}
                </p>
              )}
            </div>
          )}
        </div>
      </details>
    </PanelShell>
  );
}
