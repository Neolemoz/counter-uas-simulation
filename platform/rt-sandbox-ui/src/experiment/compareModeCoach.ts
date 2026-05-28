import type { CompareModeId } from "./experimentUnifiedReview";

export const COMPARE_MODE_COACH: Record<CompareModeId, string> = {
  pairwise_pinned:
    "Two runs — quick A/B within one manifest or one live slot + one pin.",
  extended_n_run:
    "Up to 4 runs — sweep or repeatability within one manifest.",
  cohort_matrix:
    "Parameter matrix layout — requires experiment_class = parameter_matrix.",
  multi_manifest_diff:
    "Manifest metadata only — not run outcome compare; drill into each manifest separately.",
  fidelity_compare:
    "F5b fidelity metrics — explanatory coupling lines only; not operational sensor truth.",
};

export function compareModeCoachLine(mode: CompareModeId): string {
  return COMPARE_MODE_COACH[mode];
}
