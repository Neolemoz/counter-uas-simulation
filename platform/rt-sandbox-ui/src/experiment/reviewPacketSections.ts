import { compareModeCoachLine } from "./compareModeCoach";
import type { CompareModeId } from "./experimentUnifiedReview";
import type { ExperimentManifest } from "./experimentSchema";
import type { ReportDockPresence } from "./reviewPacketPreview";
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
};

export function buildPacketSectionsPreview(options: {
  v2State: WorkbenchV2State;
  manifest: ExperimentManifest;
  presence: ReportDockPresence;
  cohortLabel?: string | null;
  coachLine?: string;
}): ReviewPacketSectionEntry[] {
  const { v2State, manifest, presence, cohortLabel, coachLine } = options;
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
  const compareBody = [
    `Mode: ${mode}.`,
    runIds.length > 0 ? `Runs: ${runIds.join(", ")}.` : "Runs: none pinned.",
    coach,
  ].join(" ");

  const cliBody = `python3 scripts/rt/rt_experiment_metrics.py --manifest runs/rt_sandbox/experiments/${manifest.experiment_id}/manifest.json`;

  return [
    {
      section_id: "scope",
      title: "Review scope",
      body_markdown: scopeBody,
      refs: v2State.active_cohort_id
        ? [`cohort://${v2State.active_cohort_id}`]
        : [],
    },
    {
      section_id: "reports",
      title: "Imported reports",
      body_markdown: reportsBody,
      refs: reportKinds.length
        ? [
            `runs/rt_sandbox/experiments/${manifest.experiment_id}/reports/analytics.json`,
          ]
        : [],
    },
    {
      section_id: "compare_summary",
      title: "Compare snapshot",
      body_markdown: compareBody,
      refs: [],
    },
    {
      section_id: "advisory_refs",
      title: "Handoff advisory",
      body_markdown: "Display refs only — no readiness verdict or SA import authority.",
      refs: [],
    },
    {
      section_id: "cli_hints",
      title: "Maintainer CLIs",
      body_markdown: cliBody,
      refs: [],
    },
  ];
}
