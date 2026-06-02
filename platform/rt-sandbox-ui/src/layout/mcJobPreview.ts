/**
 * Read-only Monte Carlo job preparation model for RT layout evaluation (UI only).
 * Does not enqueue runs, invoke scripts/monte_carlo.py, or touch the RT bridge.
 */

import type { McProfilePreview } from "./rtLayoutMcProfile";
import type {
  ScenarioEvaluationPreset,
  ScenarioEvaluationPresetId,
} from "./scenarioEvaluationPresets";

export const MC_JOB_PREVIEW_SCHEMA_VERSION = "rt_mc_job_preview_v1" as const;

/** Heuristic wall-clock seconds per MC run for maintainer planning labels only. */
export const ESTIMATED_SECONDS_PER_MC_RUN = 90;

export type McJobPreview = {
  schema_version: typeof MC_JOB_PREVIEW_SCHEMA_VERSION;
  geometry_id: string;
  preset: ScenarioEvaluationPresetId;
  preset_label: string;
  run_count: number;
  estimated_duration_sec: number;
  estimated_duration: string;
  scenario_label: string;
  launch_args: string;
  warnings: string[];
  source_layout_id: string;
  prepared_utc: string;
};

export function estimateMcJobDurationSec(runCount: number): number {
  const runs = Math.max(0, Math.floor(runCount));
  return runs * ESTIMATED_SECONDS_PER_MC_RUN;
}

/** Human-readable duration label (approximate, UI-only). */
export function formatEstimatedDuration(totalSec: number): string {
  if (totalSec <= 0) return "—";
  if (totalSec < 60) return `~${totalSec}s`;
  const minutes = Math.round(totalSec / 60);
  if (minutes < 60) return `~${minutes} min`;
  const hours = Math.floor(minutes / 60);
  const remMin = minutes % 60;
  if (remMin === 0) return `~${hours}h`;
  return `~${hours}h ${remMin}m`;
}

export function scenarioLabelFromSuggestion(suggestion: string): string {
  if (suggestion === "multi") return "multi-target";
  if (suggestion === "single") return "single-target";
  return suggestion || "unknown";
}

export function buildMcJobPreview(
  profile: McProfilePreview,
  preset: ScenarioEvaluationPreset,
  preparedUtc = new Date().toISOString().replace(/\.\d{3}Z$/, "Z"),
): McJobPreview {
  const estimated_duration_sec = estimateMcJobDurationSec(preset.runs);
  return {
    schema_version: MC_JOB_PREVIEW_SCHEMA_VERSION,
    geometry_id: profile.geometry_id,
    preset: preset.id,
    preset_label: preset.label,
    run_count: preset.runs,
    estimated_duration_sec,
    estimated_duration: formatEstimatedDuration(estimated_duration_sec),
    scenario_label: scenarioLabelFromSuggestion(profile.scenario_suggestion),
    launch_args: profile.launch_args,
    warnings: [...profile.warnings],
    source_layout_id: profile.source_layout_id,
    prepared_utc: preparedUtc,
  };
}
