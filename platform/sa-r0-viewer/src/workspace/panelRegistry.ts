import type { WorkspaceSegment } from "./types";

export type PanelId =
  | "discover.scenario_catalog"
  | "authoring.topology_inspector"
  | "authoring.validation_mirror"
  | "authoring.lineage"
  | "authoring.promotion_status"
  | "authoring.orchestration_handoff"
  | "authoring.integrity"
  | "orchestration.lifecycle"
  | "orchestration.async"
  | "orchestration.recovery"
  | "orchestration.batch_review"
  | "orchestration.integrity"
  | "orchestration.replay_continuity"
  | "discover.validation_status"
  | "discover.job_status"
  | "discover.sweep_catalog"
  | "discover.compare_catalog"
  | "corpus.browser"
  | "corpus.evolution"
  | "corpus.lineage"
  | "corpus.provenance"
  | "spatial.layers"
  | "spatial.map"
  | "temporal.scrubber"
  | "temporal.narrative"
  | "temporal.compare"
  | "narrative.annotations"
  | "narrative.patterns"
  | "metadata.bundle"
  | "analytics.sweep"
  | "workstation.sweep"
  | "workstation.experiment_review"
  | "workflow.lineage"
  | "mock.sensors"
  | "compare.controls"
  | "compare.topology"
  | "compare.outcome"
  | "compare.annotations"
  | "filmstrip.cohort"
  | "presentation.story"
  | "presentation.controls";

export type PanelMeta = {
  id: PanelId;
  tier: "t3" | "t4" | "t5";
  /** Default collapsed when segment mounts (declutter). */
  defaultCollapsed?: boolean;
  /** Primary rail visibility (else → More panels). */
  primary?: boolean;
};

const REPLAY_PRIMARY: PanelId[] = [
  "discover.scenario_catalog",
  "metadata.bundle",
  "spatial.layers",
  "temporal.scrubber",
];

const COMPARE_PRIMARY: PanelId[] = [
  "discover.compare_catalog",
  "compare.controls",
  "compare.topology",
  "compare.outcome",
  "spatial.layers",
];

const CORPUS_PRIMARY: PanelId[] = [
  "corpus.browser",
  "corpus.evolution",
  "corpus.lineage",
  "discover.sweep_catalog",
];

const SCENARIO_PRIMARY: PanelId[] = [
  "discover.scenario_catalog",
  "authoring.promotion_status",
  "authoring.topology_inspector",
  "authoring.lineage",
  "authoring.validation_mirror",
  "authoring.orchestration_handoff",
  "authoring.integrity",
  "orchestration.lifecycle",
  "orchestration.async",
  "orchestration.recovery",
  "orchestration.batch_review",
  "orchestration.integrity",
  "orchestration.replay_continuity",
  "discover.job_status",
];

const REPORT_PRIMARY: PanelId[] = ["presentation.story", "presentation.controls"];

/** Panels allowed to mount in a segment (H1 §9.2). */
export const SEGMENT_PANEL_ALLOWLIST: Record<WorkspaceSegment, PanelId[]> = {
  scenario: [
    "discover.scenario_catalog",
    "authoring.topology_inspector",
    "authoring.validation_mirror",
    "authoring.lineage",
    "authoring.promotion_status",
    "authoring.orchestration_handoff",
    "authoring.integrity",
    "orchestration.lifecycle",
    "orchestration.async",
    "orchestration.recovery",
    "orchestration.batch_review",
    "orchestration.integrity",
    "orchestration.replay_continuity",
    "discover.job_status",
    "workflow.lineage",
    "spatial.map",
  ],
  replay: [
    "discover.scenario_catalog",
    "discover.sweep_catalog",
    "discover.validation_status",
    "discover.job_status",
    "orchestration.lifecycle",
    "orchestration.integrity",
    "orchestration.replay_continuity",
    "discover.compare_catalog",
    "metadata.bundle",
    "spatial.layers",
    "temporal.scrubber",
    "temporal.narrative",
    "narrative.annotations",
    "workstation.sweep",
    "workstation.experiment_review",
    "workflow.lineage",
    "analytics.sweep",
    "mock.sensors",
    "filmstrip.cohort",
    "narrative.patterns",
  ],
  compare: [
    "discover.compare_catalog",
    "discover.scenario_catalog",
    "discover.job_status",
    "workflow.lineage",
    "compare.controls",
    "compare.topology",
    "compare.outcome",
    "compare.annotations",
    "spatial.layers",
    "temporal.compare",
    "mock.sensors",
  ],
  corpus: [
    "corpus.browser",
    "corpus.evolution",
    "corpus.lineage",
    "corpus.provenance",
    "discover.job_status",
    "discover.sweep_catalog",
    "discover.compare_catalog",
    "workflow.lineage",
  ],
  report: [
    "presentation.story",
    "presentation.controls",
    "workflow.lineage",
    "spatial.map",
    "temporal.scrubber",
    "narrative.annotations",
  ],
};

export function segmentAllowsPanel(segment: WorkspaceSegment, id: PanelId): boolean {
  return SEGMENT_PANEL_ALLOWLIST[segment].includes(id);
}

export function primaryPanelsForSegment(segment: WorkspaceSegment): PanelId[] {
  switch (segment) {
    case "scenario":
      return SCENARIO_PRIMARY;
    case "replay":
      return REPLAY_PRIMARY;
    case "compare":
      return COMPARE_PRIMARY;
    case "corpus":
      return CORPUS_PRIMARY;
    case "report":
      return REPORT_PRIMARY;
    default:
      return [];
  }
}

export function isPrimaryPanel(segment: WorkspaceSegment, id: PanelId): boolean {
  return primaryPanelsForSegment(segment).includes(id);
}

export function defaultCollapsedForPanel(segment: WorkspaceSegment, id: PanelId): boolean {
  if (id === "mock.sensors" && segment === "replay") return true;
  if (id === "analytics.sweep" && segment === "replay") return true;
  if (id === "workstation.sweep" && segment === "replay") return false;
  if (id === "workstation.experiment_review" && segment === "replay") return false;
  if (id === "discover.sweep_catalog" && segment === "replay") return true;
  if (id === "discover.compare_catalog" && segment === "replay") return true;
  if (id === "discover.job_status" && segment === "replay") return true;
  if (id === "workflow.lineage" && segment === "scenario") return true;
  if (id.startsWith("authoring.") && segment === "scenario") return false;
  if (id === "workflow.lineage" && segment === "replay") return true;
  if (id === "narrative.annotations" && segment === "replay") return true;
  if (id === "mock.sensors" && segment === "compare") return true;
  return !isPrimaryPanel(segment, id);
}

/** Max expanded section headers in left rail (H1 cognitive load budget). */
export const MAX_PRIMARY_RAIL_SECTIONS = 4;
