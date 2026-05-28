import type { CaptureHandoffRow } from "@/bridge/types";
import { enrichAdvisoryRow } from "./advisoryAggregate";
import type {
  BlockerGroupId,
  EnrichedAdvisoryRow,
  HandoffRollupByStage,
  QueueBandId,
  ReadinessCohortId,
  ReadinessCohortV2Id,
  TriageGroupMode,
  TriageRowGroup,
  TriageSortMode,
  AdvisoryExperimentRollup,
} from "./advisoryTypes";
import { queueBandLabel } from "./advisoryQueue";

const QUEUE_BAND_ORDER: QueueBandId[] = [
  "P0_block",
  "P1_error",
  "P2_normalize",
  "P3_review",
  "P4_approve",
  "P5_package",
  "P6_import",
  "P7_terminal",
];

const BLOCKER_GROUP_ORDER: BlockerGroupId[] = [
  "normalization",
  "review_attestation",
  "approval_gate",
  "packaging",
  "lineage",
  "experiment_warn",
  "terminal_block",
];

const COHORT_ORDER: ReadinessCohortId[] = [
  "error",
  "blocked",
  "needs_normalize",
  "needs_review",
  "needs_approve",
  "needs_prepare",
  "ready_for_commit_advisory",
  "terminal",
];

const COHORT_V2_ORDER: ReadinessCohortV2Id[] = [
  "focus_highlight",
  "experiment_handoff_warn",
  "multi_blocker",
  "stale_review",
  "stale_approve",
  "error",
  "blocked",
  "needs_normalize",
  "needs_review",
  "needs_approve",
  "needs_prepare",
  "ready_for_commit_advisory",
  "terminal",
];

const HANDOFF_STAGE_ORDER: (keyof HandoffRollupByStage | "blocked" | "terminal")[] = [
  "normalize",
  "review",
  "approve",
  "prepare",
  "import_advisory",
  "blocked",
  "terminal",
];

export function handoffStageKeyForRow(
  row: EnrichedAdvisoryRow,
): keyof HandoffRollupByStage | "blocked" | "terminal" {
  const cohort = row.readiness_cohort;
  if (cohort === "blocked") return "blocked";
  if (cohort === "terminal") return "terminal";
  const map: Partial<Record<ReadinessCohortId, keyof HandoffRollupByStage>> = {
    needs_normalize: "normalize",
    needs_review: "review",
    needs_approve: "approve",
    needs_prepare: "prepare",
    ready_for_commit_advisory: "import_advisory",
    error: "review",
  };
  return map[cohort] ?? "review";
}

export function primaryBlockerGroup(groups: BlockerGroupId[]): BlockerGroupId | null {
  for (const g of BLOCKER_GROUP_ORDER) {
    if (groups.includes(g)) return g;
  }
  return null;
}

function sortRows(rows: EnrichedAdvisoryRow[], sort: TriageSortMode): EnrichedAdvisoryRow[] {
  const copy = [...rows];
  if (sort === "capture_id") {
    return copy.sort((a, b) =>
      a.capture_candidate_id.localeCompare(b.capture_candidate_id),
    );
  }
  return copy.sort(
    (a, b) =>
      a.queue_priority.rank - b.queue_priority.rank ||
      a.capture_candidate_id.localeCompare(b.capture_candidate_id),
  );
}

function pushGroup(
  map: Map<string, TriageRowGroup>,
  key: string,
  label: string,
  row: EnrichedAdvisoryRow,
): void {
  const existing = map.get(key);
  if (existing) {
    existing.rows.push(row);
  } else {
    map.set(key, { key, label, rows: [row] });
  }
}

export function groupEnrichedRows(
  rows: EnrichedAdvisoryRow[],
  mode: TriageGroupMode,
  options?: {
    sort?: TriageSortMode;
    experimentRollup?: AdvisoryExperimentRollup | null;
  },
): TriageRowGroup[] {
  const sort = options?.sort ?? "queue";
  const sorted = sortRows(rows, sort);
  const map = new Map<string, TriageRowGroup>();

  if (mode === "queue_band") {
    for (const row of sorted) {
      const band = row.queue_priority.band as QueueBandId;
      pushGroup(map, band, queueBandLabel(band), row);
    }
    return QUEUE_BAND_ORDER.filter((b) => map.has(b)).map((b) => map.get(b)!);
  }

  if (mode === "blocker") {
    for (const row of sorted) {
      if (!row.blocker_groups.length) {
        pushGroup(map, "none", "no blocker group", row);
        continue;
      }
      for (const g of row.blocker_groups) {
        pushGroup(map, g, g.replace(/_/g, " "), row);
      }
    }
    const ordered = BLOCKER_GROUP_ORDER.filter((g) => map.has(g)).map((g) => map.get(g)!);
    if (map.has("none")) ordered.push(map.get("none")!);
    return ordered;
  }

  if (mode === "cohort") {
    for (const row of sorted) {
      const c = row.readiness_cohort;
      pushGroup(map, c, c.replace(/_/g, " "), row);
    }
    return COHORT_ORDER.filter((c) => map.has(c)).map((c) => map.get(c)!);
  }

  if (mode === "cohort_v2") {
    for (const row of sorted) {
      const c = row.readiness_cohort_v2 ?? row.readiness_cohort;
      pushGroup(map, c, `v2: ${c.replace(/_/g, " ")}`, row);
    }
    const ordered = COHORT_V2_ORDER.filter((c) => map.has(c)).map((c) => map.get(c)!);
    for (const [key, group] of map) {
      if (!COHORT_V2_ORDER.includes(key as ReadinessCohortV2Id)) {
        ordered.push(group);
      }
    }
    return ordered;
  }

  if (mode === "handoff_stage") {
    for (const row of sorted) {
      const stage = handoffStageKeyForRow(row);
      pushGroup(map, stage, `stage: ${stage.replace(/_/g, " ")}`, row);
    }
    return HANDOFF_STAGE_ORDER.filter((s) => map.has(s)).map((s) => map.get(s)!);
  }

  // experiment — warn-only rollup
  const warnSet = new Set(options?.experimentRollup?.warn_capture_ids ?? []);
  const experimentRows = sorted.filter(
    (r) =>
      warnSet.has(r.capture_candidate_id) ||
      r.blocker_groups.includes("experiment_warn"),
  );
  if (experimentRows.length === 0) {
    return [];
  }
  pushGroup(
    map,
    "experiment_warn",
    "experiment advisory (warn-only)",
    experimentRows[0],
  );
  const group = map.get("experiment_warn")!;
  group.rows = experimentRows;
  return [group];
}

export function formatTriageStandUpSummary(groups: TriageRowGroup[]): string {
  const lines: string[] = [
    "RT advisory triage (read-only) — maintainer CLIs are authority",
    "",
  ];
  for (const g of groups) {
    lines.push(`## ${g.label}`);
    for (const row of g.rows) {
      const state = row.status.terminal
        ? "committed"
        : row.status.advisory_state ?? "pending";
      const blockers = row.blocker_groups.join(", ") || "—";
      const lineage = row.status.lineage_warnings?.join("; ") ?? "";
      lines.push(
        `- [${row.queue_priority.band}] ${row.capture_candidate_id} | ${state} | blockers: ${blockers}${lineage ? ` | lineage: ${lineage}` : ""}`,
      );
    }
    lines.push("");
  }
  return lines.join("\n").trimEnd();
}

export function rollupBlockerGroupExemplars(
  rows: EnrichedAdvisoryRow[],
  max = 5,
): Partial<Record<BlockerGroupId, { count: number; exemplar_capture_ids: string[] }>> {
  const idsByGroup: Partial<Record<BlockerGroupId, string[]>> = {};
  for (const row of rows) {
    for (const g of row.blocker_groups) {
      const list = idsByGroup[g] ?? [];
      if (list.length < max) {
        list.push(row.capture_candidate_id);
        idsByGroup[g] = list;
      }
    }
  }
  const out: Partial<
    Record<BlockerGroupId, { count: number; exemplar_capture_ids: string[] }>
  > = {};
  for (const g of BLOCKER_GROUP_ORDER) {
    const ids = idsByGroup[g];
    if (!ids?.length) continue;
    const count = rows.filter((r) => r.blocker_groups.includes(g)).length;
    out[g] = { count, exemplar_capture_ids: ids };
  }
  return out;
}

export function defaultBandOpen(band: string): boolean {
  return band === "P0_block" || band === "P1_error" || band === "P2_normalize";
}

export function buildExperimentRollupFromWorkbench(
  experimentLevel: string,
  warnCaptureIds: string[],
): AdvisoryExperimentRollup {
  return {
    handoff_eligibility: experimentLevel,
    warn_capture_ids: warnCaptureIds,
    note: "experiment eligibility is warn-only; per-capture advisory is authority",
  };
}

export function warnCaptureIdsFromHandoffAndMetrics(
  handoffRows: CaptureHandoffRow[],
  sessionLifecycleState: string | null | undefined,
  experimentLevel: string,
  manifestCaptureHints: Map<string, string | undefined>,
): string[] {
  if (experimentLevel === "ineligible") return [];
  const warnIds: string[] = [];
  for (const row of handoffRows) {
    const enriched = enrichAdvisoryRow(row, sessionLifecycleState, {
      experimentWarn: false,
    });
    const hint = manifestCaptureHints.get(row.capture_candidate_id);
    const expEligible =
      hint === "eligible" || experimentLevel === "eligible" || experimentLevel === "partial";
    if (
      expEligible &&
      enriched.status.advisory_state !== "import_ready" &&
      !enriched.status.terminal
    ) {
      warnIds.push(row.capture_candidate_id);
    }
  }
  return [...new Set(warnIds)];
}
