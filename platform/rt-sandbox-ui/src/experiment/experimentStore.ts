import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type { TelemetryChannel } from "@/telemetry/constants";
import { nearestRidgeLabel } from "@/cesium/rtFictionalTerrain";
import { safeParseManifest } from "./experimentImportGuards";
import {
  EXPERIMENT_GOVERNANCE_BANNER,
  experimentManifestSchema,
  type ExperimentManifest,
  type ExperimentRun,
  type TacticalAnnexSummary,
} from "./experimentSchema";

const STORAGE_KEY = "rt_experiment_manifest_v1";

function utcNow(): string {
  return new Date().toISOString().replace(/\.\d{3}Z$/, "Z");
}

export function createEmptyManifest(experimentId: string): ExperimentManifest {
  return {
    schema: "rt_experiment_manifest_v1",
    experiment_id: experimentId,
    created_at_utc: utcNow(),
    governance_banner: EXPERIMENT_GOVERNANCE_BANNER,
    runs: [],
  };
}

export function loadManifestFromStorage(): ExperimentManifest | null {
  if (typeof localStorage === "undefined") return null;
  const raw = localStorage.getItem(STORAGE_KEY);
  if (!raw) return null;
  const parsed = safeParseManifest(raw);
  return parsed.ok ? parsed.data : null;
}

export function saveManifestToStorage(manifest: ExperimentManifest): void {
  const parsed = experimentManifestSchema.parse(manifest);
  localStorage.setItem(STORAGE_KEY, JSON.stringify(parsed, null, 2));
}

export function parseManifestJson(text: string): ExperimentManifest {
  return experimentManifestSchema.parse(JSON.parse(text));
}

export function exportManifestJson(manifest: ExperimentManifest): string {
  return JSON.stringify(experimentManifestSchema.parse(manifest), null, 2);
}

export function manifestExportPath(experimentId: string): string {
  return `runs/rt_sandbox/experiments/${experimentId}/manifest.json`;
}

export function buildTerrainContext(
  layersEnabled: boolean,
  sampleX = 0,
  sampleY = 0,
  options?: {
    contourLayersOn?: boolean;
    visibilityHint?: string | null;
  },
): ExperimentRun["snapshot"]["terrain_context"] {
  return {
    layers_enabled: layersEnabled,
    nearest_ridge: layersEnabled ? nearestRidgeLabel(sampleX, sampleY) : null,
    contour_layers_on: options?.contourLayersOn ?? false,
    visibility_hint: layersEnabled ? (options?.visibilityHint ?? undefined) : undefined,
    note: "Fictional RT terrain fixture — same profile for all sessions",
  };
}

export function pinRunSnapshot(options: {
  manifest: ExperimentManifest;
  runId: string;
  label: string;
  sessionId: string;
  snapshots: Partial<Record<TelemetryChannel, ChannelSnapshot>>;
  terrainLayersEnabled?: boolean;
  terrainContourOn?: boolean;
  terrainVisibilityHint?: string | null;
  captureCandidateId?: string | null;
  captureStagingRef?: string | null;
  tacticalAnnexSummary?: TacticalAnnexSummary | null;
}): ExperimentManifest {
  const snapPayload = (ch: TelemetryChannel) =>
    options.snapshots[ch]?.payload ?? undefined;

  const run: ExperimentRun = {
    run_id: options.runId,
    label: options.label,
    session_id: options.sessionId,
    recorded_at_utc: utcNow(),
    capture_candidate_id: options.captureCandidateId ?? null,
    capture_staging_ref: options.captureStagingRef ?? null,
    snapshot: {
      tactical_state: snapPayload("tactical_state"),
      world_summary: snapPayload("world_summary"),
      lifecycle_state: snapPayload("lifecycle_state"),
      entity_pose_mirror: snapPayload("entity_pose_mirror"),
      terrain_context: buildTerrainContext(options.terrainLayersEnabled ?? false, 0, 0, {
        contourLayersOn: options.terrainContourOn,
        visibilityHint: options.terrainVisibilityHint,
      }),
    },
    tactical_annex_summary: options.tacticalAnnexSummary ?? null,
  };

  const runs = options.manifest.runs.filter((r) => r.run_id !== options.runId);
  runs.push(run);
  return experimentManifestSchema.parse({
    ...options.manifest,
    runs,
  });
}

export function removeRun(manifest: ExperimentManifest, runId: string): ExperimentManifest {
  return experimentManifestSchema.parse({
    ...manifest,
    runs: manifest.runs.filter((r) => r.run_id !== runId),
  });
}

export function annexSummaryFromJson(data: unknown): TacticalAnnexSummary | null {
  if (!data || typeof data !== "object") return null;
  const d = data as Record<string, unknown>;
  return {
    final_tactical_mode: (d.final_tactical_mode as string) ?? null,
    selected_id: (d.selected_id as string) ?? null,
    assigned_target: (d.assigned_target as string) ?? null,
    timeline_counts: {
      mode_switches: Array.isArray(d.mode_switches) ? d.mode_switches.length : 0,
      assignment_timeline: Array.isArray(d.assignment_timeline)
        ? d.assignment_timeline.length
        : 0,
      pause_resume_transitions: Array.isArray(d.pause_resume_transitions)
        ? d.pause_resume_transitions.length
        : 0,
      recommendation_timeline: Array.isArray(d.recommendation_timeline)
        ? d.recommendation_timeline.length
        : 0,
    },
  };
}
