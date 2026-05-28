import type { CohortManifestRef } from "./cohortSchema";
import type { ExperimentManifest } from "./experimentSchema";

export type ManifestRefStatus = "ok" | "missing" | "id_mismatch";

export function deriveManifestRefStatus(
  ref: CohortManifestRef,
  loadedManifest: ExperimentManifest | null,
  isPrimaryOrSecondary: boolean,
): ManifestRefStatus {
  if (!isPrimaryOrSecondary) return "ok";
  if (!loadedManifest) return "missing";
  if (loadedManifest.experiment_id !== ref.experiment_id) return "id_mismatch";
  return "ok";
}

export function runCountLabel(
  ref: CohortManifestRef,
  loadedManifest: ExperimentManifest | null,
): string {
  if (loadedManifest && loadedManifest.experiment_id === ref.experiment_id) {
    return String(loadedManifest.runs.length);
  }
  if (ref.run_count_hint != null) return `~${ref.run_count_hint}`;
  return "—";
}
