import type { CompareModeId, UnifiedReviewStepId } from "./experimentUnifiedReview";

export type StepPanelTargets = {
  analytics: boolean;
  continuity: boolean;
  compare: boolean;
  f5: boolean;
};

export type CompareModeActivations = {
  compareModeActive: boolean;
  f5Active: boolean;
};

export function stepPanelTargets(step: UnifiedReviewStepId): StepPanelTargets {
  switch (step) {
    case "f1_analytics":
      return { analytics: true, continuity: false, compare: false, f5: false };
    case "f3_continuity":
      return { analytics: false, continuity: true, compare: false, f5: false };
    case "f5_metrics":
    case "f5b_fidelity":
      return { analytics: false, continuity: false, compare: false, f5: true };
    case "compare":
      return { analytics: false, continuity: false, compare: true, f5: false };
    default:
      return { analytics: false, continuity: false, compare: false, f5: false };
  }
}

export function compareModeActivations(mode: CompareModeId): CompareModeActivations {
  switch (mode) {
    case "pairwise_pinned":
      return { compareModeActive: true, f5Active: false };
    case "extended_n_run":
    case "cohort_matrix":
    case "fidelity_compare":
      return { compareModeActive: false, f5Active: true };
    case "multi_manifest_diff":
      return { compareModeActive: false, f5Active: false };
    default:
      return { compareModeActive: true, f5Active: false };
  }
}

export function applyStepWithCompareMode(
  step: UnifiedReviewStepId,
  compareMode: CompareModeId,
): StepPanelTargets & CompareModeActivations {
  const base = stepPanelTargets(step);
  if (step !== "compare") {
    return { ...base, compareModeActive: false, f5Active: base.f5 };
  }
  const cmp = compareModeActivations(compareMode);
  return {
    ...base,
    compare: cmp.compareModeActive,
    f5: cmp.f5Active,
    ...cmp,
  };
}
