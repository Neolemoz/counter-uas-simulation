export const UNIFIED_REVIEW_STEPS = [
  {
    id: "select_scope",
    label: "Select scope",
    panelHint: "Import cohort index or single manifest (frozen workbench below)",
  },
  {
    id: "f1_analytics",
    label: "F1 analytics",
    panelHint: "Experiment analytics panel — deriveExperimentAnalytics / rt_experiment_analytics.py",
  },
  {
    id: "f3_continuity",
    label: "F3 continuity",
    panelHint: "Continuity review panel — optional annex cache per run",
  },
  {
    id: "f5_metrics",
    label: "F5 metrics",
    panelHint: "Advanced experiment metrics (F5) — rt_experiment_metrics_report_v1",
  },
  {
    id: "f5b_fidelity",
    label: "F5b fidelity",
    panelHint: "Fidelity metrics — rt_experiment_fidelity_metrics_report_v1",
  },
  {
    id: "compare",
    label: "Compare",
    panelHint: "Compare stage — frozen X1/F5 panels below",
  },
  {
    id: "export_packet",
    label: "Export packet",
    panelHint: "rt_experiment_review_packet_v1 — preview + file export in report dock",
  },
] as const;

export type UnifiedReviewStepId = (typeof UNIFIED_REVIEW_STEPS)[number]["id"];

export const REPORT_DOCK_SLOTS = [
  {
    id: "f1_analytics",
    label: "F1 analytics",
    schema: "rt_experiment_analytics_report_v1",
  },
  {
    id: "f3_annex",
    label: "F3 annex cache",
    schema: "rt_experiment_annex_cache_v1",
  },
  {
    id: "f5_metrics",
    label: "F5 metrics",
    schema: "rt_experiment_metrics_report_v1",
  },
  {
    id: "f5b_fidelity",
    label: "F5b fidelity",
    schema: "rt_experiment_fidelity_metrics_report_v1",
  },
] as const;

export type ReportDockSlotId = (typeof REPORT_DOCK_SLOTS)[number]["id"];

/** Contract modes from rt_experiment_compare_workflow_v2_v1 */
export const COMPARE_MODES_CONTRACT = [
  { id: "pairwise_pinned", label: "Pinned (X1 pairwise)" },
  { id: "extended_n_run", label: "Side-by-side (F5 extended)" },
  { id: "cohort_matrix", label: "Matrix (F5 parameter)" },
  { id: "multi_manifest_diff", label: "Multi-manifest diff (metadata)" },
] as const;

/** P1 UI-only fidelity compare host */
export const COMPARE_MODE_FIDELITY = "fidelity_compare" as const;

export type CompareModeId =
  | (typeof COMPARE_MODES_CONTRACT)[number]["id"]
  | typeof COMPARE_MODE_FIDELITY;

export const COMPARE_MODE_UI = [
  { id: "pairwise_pinned" as const, label: "Pinned" },
  { id: "extended_n_run" as const, label: "Side-by-side" },
  { id: "cohort_matrix" as const, label: "Matrix" },
  { id: "fidelity_compare" as const, label: "Fidelity" },
  { id: "multi_manifest_diff" as const, label: "Multi-manifest" },
] as const;

export function compareModeLabel(mode: CompareModeId): string {
  const ui = COMPARE_MODE_UI.find((m) => m.id === mode);
  if (ui) return ui.label;
  return COMPARE_MODES_CONTRACT.find((m) => m.id === mode)?.label ?? mode;
}

export const REVIEW_STEP_IDS = UNIFIED_REVIEW_STEPS.map((s) => s.id);
