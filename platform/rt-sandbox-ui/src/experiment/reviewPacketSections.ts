import { compareModeCoachLine } from "./compareModeCoach";
import type { CompareModeId, UnifiedReviewStepId } from "./experimentUnifiedReview";
import type { ExperimentManifest } from "./experimentSchema";
import type { ReportDockPresence } from "./reviewPacketPreview";
import type { ReviewStepCompletionState } from "./reviewStepCompletion";
import type { WorkbenchV2State } from "./workbenchV2State";

export type ReviewPacketSectionId =
  | "scope"
  | "reports"
  | "compare_summary"
  | "advisory_refs"
  | "cli_hints";

export const PACKET_SECTION_CATALOG: ReadonlyArray<{
  section_id: ReviewPacketSectionId;
  title: string;
}> = [
  { section_id: "scope", title: "Review scope" },
  { section_id: "reports", title: "Imported reports" },
  { section_id: "compare_summary", title: "Compare snapshot" },
  { section_id: "advisory_refs", title: "Handoff advisory" },
  { section_id: "cli_hints", title: "Maintainer CLIs" },
];

export type ReviewPacketSectionEntry = {
  section_id: ReviewPacketSectionId;
  title: string;
  body_markdown: string;
  refs: string[];
  completion_hint?: ReviewStepCompletionState;
};

const SECTION_STEP: Record<ReviewPacketSectionId, UnifiedReviewStepId> = {
  scope: "select_scope",
  reports: "f1_analytics",
  compare_summary: "compare",
  advisory_refs: "export_packet",
  cli_hints: "export_packet",
};

export function buildPacketSectionsPreview(options: {
  v2State: WorkbenchV2State;
  manifest: ExperimentManifest;
  presence: ReportDockPresence;
  cohortLabel?: string | null;
  coachLine?: string;
  stepCompletion?: Partial<Record<UnifiedReviewStepId, ReviewStepCompletionState>>;
  multiManifestStatusLine?: string;
}): ReviewPacketSectionEntry[] {
  const {
    v2State,
    manifest,
    presence,
    cohortLabel,
    coachLine,
    stepCompletion,
    multiManifestStatusLine,
  } = options;
  const cohortPart = cohortLabel ?? v2State.active_cohort_id ?? "single manifest";
  const primaryRef = v2State.primary_manifest_ref ?? manifest.experiment_id;
  const secondaryRef = v2State.secondary_manifest_ref;
  const runPart = v2State.active_run_id ? ` → run ${v2State.active_run_id}` : "";

  const scopeBody = [
    `Cohort: ${cohortPart}.`,
    `Primary: ${primaryRef}.`,
    secondaryRef ? `Secondary: ${secondaryRef}.` : "Secondary: none.",
    runPart ? `Active run focus${runPart}.` : "",
    `Review session: ${v2State.review_session_id}.`,
  ]
    .filter(Boolean)
    .join(" ");

  const reportKinds: string[] = [];
  if (presence.f1_analytics) reportKinds.push("F1 analytics");
  if (presence.f3_annex) reportKinds.push("F3 annex cache");
  if (presence.f5_metrics) reportKinds.push("F5 metrics");
  if (presence.f5b_fidelity) reportKinds.push("F5b fidelity");
  const reportsBody =
    reportKinds.length > 0
      ? `Imported: ${reportKinds.join(", ")}.`
      : "No imported reports in dock slots.";

  const mode = v2State.compare_mode as CompareModeId;
  const coach =
    coachLine ?? compareModeCoachLine(mode);
  const runIds = [v2State.compare_run_a, v2State.compare_run_b].filter(
    (id): id is string => typeof id === "string" && id.length > 0,
  );
  const compareParts = [
    `Mode: ${mode}.`,
    runIds.length > 0 ? `Runs: ${runIds.join(", ")}.` : "Runs: none pinned.",
    coach,
  ];
  if (mode === "multi_manifest_diff") {
    if (primaryRef) compareParts.push(`Primary manifest: ${primaryRef}.`);
    if (secondaryRef) compareParts.push(`Secondary manifest: ${secondaryRef}.`);
    if (multiManifestStatusLine) {
      compareParts.push(`Metadata status: ${multiManifestStatusLine}.`);
    }
  }
  const compareBody = compareParts.join(" ");

  const cliBody = `python3 scripts/rt/rt_experiment_metrics.py --manifest runs/rt_sandbox/experiments/${manifest.experiment_id}/manifest.json`;

  const withHint = (
    section_id: ReviewPacketSectionId,
    title: string,
    body_markdown: string,
    refs: string[],
  ): ReviewPacketSectionEntry => ({
    section_id,
    title,
    body_markdown,
    refs,
    completion_hint: stepCompletion?.[SECTION_STEP[section_id]],
  });

  return [
    withHint(
      "scope",
      "Review scope",
      scopeBody,
      v2State.active_cohort_id ? [`cohort://${v2State.active_cohort_id}`] : [],
    ),
    withHint(
      "reports",
      "Imported reports",
      reportsBody,
      reportKinds.length
        ? [`runs/rt_sandbox/experiments/${manifest.experiment_id}/reports/analytics.json`]
        : [],
    ),
    withHint("compare_summary", "Compare snapshot", compareBody, []),
    withHint(
      "advisory_refs",
      "Handoff advisory",
      "Display refs only — no readiness verdict or SA import authority.",
      [],
    ),
    withHint("cli_hints", "Maintainer CLIs", cliBody, []),
  ];
}
