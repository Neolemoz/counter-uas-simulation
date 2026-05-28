import type { CompareModeId, UnifiedReviewStepId } from "./experimentUnifiedReview";

export const WORKBENCH_V2_STATE_KEY = "rt_experiment_workbench_v2_state_v1";

export type WorkbenchV2State = {
  active_cohort_id: string | null;
  primary_manifest_ref: string | null;
  secondary_manifest_ref: string | null;
  review_step: UnifiedReviewStepId;
  compare_mode: CompareModeId;
  active_run_id: string | null;
  compare_run_a: string | null;
  compare_run_b: string | null;
  steps_completed: UnifiedReviewStepId[];
};

export function defaultWorkbenchV2State(): WorkbenchV2State {
  return {
    active_cohort_id: null,
    primary_manifest_ref: null,
    secondary_manifest_ref: null,
    review_step: "select_scope",
    compare_mode: "pairwise_pinned",
    active_run_id: null,
    compare_run_a: null,
    compare_run_b: null,
    steps_completed: [],
  };
}

export function loadWorkbenchV2State(): WorkbenchV2State {
  if (typeof localStorage === "undefined") return defaultWorkbenchV2State();
  const raw = localStorage.getItem(WORKBENCH_V2_STATE_KEY);
  if (!raw) return defaultWorkbenchV2State();
  try {
    const parsed = JSON.parse(raw) as Partial<WorkbenchV2State>;
    return {
      ...defaultWorkbenchV2State(),
      ...parsed,
      steps_completed: Array.isArray(parsed.steps_completed)
        ? parsed.steps_completed
        : [],
    };
  } catch {
    return defaultWorkbenchV2State();
  }
}

export function saveWorkbenchV2State(state: WorkbenchV2State): void {
  if (typeof localStorage === "undefined") return;
  localStorage.setItem(WORKBENCH_V2_STATE_KEY, JSON.stringify(state, null, 2));
}

export function setReviewStep(
  state: WorkbenchV2State,
  step: UnifiedReviewStepId,
): WorkbenchV2State {
  return { ...state, review_step: step };
}

export function selectPrimaryManifestRef(
  state: WorkbenchV2State,
  manifestRef: string | null,
): WorkbenchV2State {
  return { ...state, primary_manifest_ref: manifestRef };
}

export function selectSecondaryManifestRef(
  state: WorkbenchV2State,
  manifestRef: string | null,
): WorkbenchV2State {
  return { ...state, secondary_manifest_ref: manifestRef };
}

export function setActiveCohort(
  state: WorkbenchV2State,
  cohortId: string | null,
): WorkbenchV2State {
  return { ...state, active_cohort_id: cohortId };
}

export function setCompareMode(
  state: WorkbenchV2State,
  mode: CompareModeId,
): WorkbenchV2State {
  return { ...state, compare_mode: mode };
}

export function setActiveRunId(
  state: WorkbenchV2State,
  runId: string | null,
): WorkbenchV2State {
  return { ...state, active_run_id: runId };
}

export function setCompareRuns(
  state: WorkbenchV2State,
  runA: string | null,
  runB: string | null,
): WorkbenchV2State {
  return { ...state, compare_run_a: runA, compare_run_b: runB };
}

export function markStepCompleted(
  state: WorkbenchV2State,
  step: UnifiedReviewStepId,
): WorkbenchV2State {
  if (state.steps_completed.includes(step)) return state;
  return { ...state, steps_completed: [...state.steps_completed, step] };
}

export function advanceReviewStep(state: WorkbenchV2State): WorkbenchV2State {
  const ids = [
    "select_scope",
    "f1_analytics",
    "f3_continuity",
    "f5_metrics",
    "f5b_fidelity",
    "compare",
    "export_packet",
  ] as const;
  const idx = ids.indexOf(state.review_step);
  if (idx < 0 || idx >= ids.length - 1) return state;
  const next = ids[idx + 1];
  return markStepCompleted({ ...state, review_step: next }, state.review_step);
}

export function retreatReviewStep(state: WorkbenchV2State): WorkbenchV2State {
  const ids = [
    "select_scope",
    "f1_analytics",
    "f3_continuity",
    "f5_metrics",
    "f5b_fidelity",
    "compare",
    "export_packet",
  ] as const;
  const idx = ids.indexOf(state.review_step);
  if (idx <= 0) return state;
  return { ...state, review_step: ids[idx - 1] };
}
