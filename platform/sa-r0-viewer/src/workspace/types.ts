/** PLAN-SA-H1 / H2 workspace segments — reviewer task contexts, not operational modes. */
export type WorkspaceSegment = "scenario" | "replay" | "compare" | "corpus" | "report";

export type WorkspaceLayoutVariant =
  | "segment"
  | "publication"
  | "filmstrip"
  | "compare"
  | "focus";

export type WorkspaceTier = "t0" | "t1" | "t2" | "t3" | "t4" | "t5" | "t6" | "t7";

export const SEGMENT_ORDER: WorkspaceSegment[] = [
  "scenario",
  "replay",
  "compare",
  "corpus",
  "report",
];

export const SEGMENT_LABELS: Record<WorkspaceSegment, string> = {
  scenario: "Scenario",
  replay: "Replay",
  compare: "Compare",
  corpus: "Corpus",
  report: "Report",
};
