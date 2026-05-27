import type {
  ComparePairAnalytics,
  ExperimentRun,
  HandoffEligibility,
  PerRunExtended,
} from "./experimentSchema";

export const MATRIX_REVIEW_CLASSES = [
  "parameter_matrix",
  "terrain_comparison",
  "sensor_range_comparison",
] as const;

export type MatrixReviewClass = (typeof MATRIX_REVIEW_CLASSES)[number];

export const F5_FILTER_ALL = "__all__";

export type F5Filters = {
  experiment_class: string;
  tactical_mode: string;
  terrain_preset: string;
  visibility_context: string;
};

export const EMPTY_F5_FILTERS: F5Filters = {
  experiment_class: F5_FILTER_ALL,
  tactical_mode: F5_FILTER_ALL,
  terrain_preset: F5_FILTER_ALL,
  visibility_context: F5_FILTER_ALL,
};

export type HandoffDisplayLevel = "eligible" | "review_needed" | "blocked";

export function handoffDisplayLevel(
  level: HandoffEligibility["experiment_level"],
): HandoffDisplayLevel {
  if (level === "eligible") return "eligible";
  if (level === "partial") return "review_needed";
  return "blocked";
}

export function extendedForRun(
  perRun: PerRunExtended[],
  runId: string,
): PerRunExtended | undefined {
  return perRun.find((r) => r.run_id === runId);
}

export function terrainPresetForExtended(ext: PerRunExtended): string | null {
  return ext.terrain_profile_ref ?? ext.nearest_ridge ?? null;
}

export function filterManifestRuns(
  runs: ExperimentRun[],
  filters: F5Filters,
  perRunExtended: PerRunExtended[],
): ExperimentRun[] {
  return runs.filter((run) => {
    const ext = extendedForRun(perRunExtended, run.run_id);
    const cls = run.experiment_class ?? ext?.experiment_class ?? null;
    if (
      filters.experiment_class !== F5_FILTER_ALL &&
      cls !== filters.experiment_class
    ) {
      return false;
    }
    if (
      filters.tactical_mode !== F5_FILTER_ALL &&
      (ext?.mode_at_capture ?? null) !== filters.tactical_mode
    ) {
      return false;
    }
    if (filters.terrain_preset !== F5_FILTER_ALL) {
      const preset = ext ? terrainPresetForExtended(ext) : run.terrain_profile_ref ?? null;
      if (preset !== filters.terrain_preset) return false;
    }
    if (
      filters.visibility_context !== F5_FILTER_ALL &&
      (ext?.los_cognition_label ?? null) !== filters.visibility_context
    ) {
      return false;
    }
    return true;
  });
}

export function collectMatrixAxisKeys(runs: ExperimentRun[]): string[] {
  const keys = new Set<string>();
  for (const run of runs) {
    if (!run.matrix_coords) continue;
    for (const k of Object.keys(run.matrix_coords)) {
      keys.add(k);
    }
  }
  return [...keys].sort();
}

export type MatrixCell = {
  rowValue: string;
  colValue: string;
  runId: string | null;
};

export function buildMatrixGrid(
  runs: ExperimentRun[],
  axisRow: string,
  axisCol: string,
): { rows: string[]; cols: string[]; cells: MatrixCell[] } {
  const rowVals = new Set<string>();
  const colVals = new Set<string>();
  const byKey = new Map<string, string>();

  for (const run of runs) {
    const coords = run.matrix_coords;
    if (!coords) continue;
    const rv = coords[axisRow];
    const cv = coords[axisCol];
    if (rv === undefined || cv === undefined) continue;
    rowVals.add(rv);
    colVals.add(cv);
    byKey.set(`${rv}\0${cv}`, run.run_id);
  }

  const rows = [...rowVals].sort();
  const cols = [...colVals].sort();
  const cells: MatrixCell[] = [];
  for (const rowValue of rows) {
    for (const colValue of cols) {
      cells.push({
        rowValue,
        colValue,
        runId: byKey.get(`${rowValue}\0${colValue}`) ?? null,
      });
    }
  }
  return { rows, cols, cells };
}

export function pairsForRuns(
  comparePairs: ComparePairAnalytics[],
  runIds: string[],
): ComparePairAnalytics[] {
  const set = new Set(runIds);
  return comparePairs.filter(
    (p) => set.has(p.run_id_a) && set.has(p.run_id_b),
  );
}

export function isMatrixReviewClass(
  experimentClass: string,
): experimentClass is MatrixReviewClass {
  return (MATRIX_REVIEW_CLASSES as readonly string[]).includes(experimentClass);
}

export function collectFilterOptions(
  runs: ExperimentRun[],
  perRunExtended: PerRunExtended[],
): {
  experiment_class: string[];
  tactical_mode: string[];
  terrain_preset: string[];
  visibility_context: string[];
} {
  const experiment_class = new Set<string>();
  const tactical_mode = new Set<string>();
  const terrain_preset = new Set<string>();
  const visibility_context = new Set<string>();

  for (const run of runs) {
    const ext = extendedForRun(perRunExtended, run.run_id);
    const cls = run.experiment_class ?? ext?.experiment_class;
    if (cls) experiment_class.add(cls);
    if (ext?.mode_at_capture) tactical_mode.add(ext.mode_at_capture);
    const preset = ext ? terrainPresetForExtended(ext) : run.terrain_profile_ref;
    if (preset) terrain_preset.add(preset);
    if (ext?.los_cognition_label) visibility_context.add(ext.los_cognition_label);
  }

  return {
    experiment_class: [...experiment_class].sort(),
    tactical_mode: [...tactical_mode].sort(),
    terrain_preset: [...terrain_preset].sort(),
    visibility_context: [...visibility_context].sort(),
  };
}

export function toggleExtendedCompareRun(
  current: string[],
  runId: string,
  max = 4,
): string[] {
  if (current.includes(runId)) {
    return current.filter((id) => id !== runId);
  }
  if (current.length >= max) {
    return [...current.slice(1), runId];
  }
  return [...current, runId];
}
