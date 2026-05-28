import type { CaptureHandoffRow } from "@/bridge/types";
import { enrichAdvisoryRowV2 } from "./advisoryAggregationV2";
import {
  classifyBlockerGroups,
  computeQueuePriority,
  readinessCohort,
  sortEnrichedByQueue,
} from "./advisoryQueue";
import { advisoryInputFromMirrorRow } from "./advisoryInputFromMirrorRow";
import { deriveAdvisoryState } from "./deriveAdvisoryState";
import type {
  AdvisoryState,
  BlockerGroupId,
  EnrichedAdvisoryRow,
  ReadinessCohortId,
  SessionAdvisorySummary,
} from "./advisoryTypes";

export function advisoryCountsSummary(
  rows: CaptureHandoffRow[],
  sessionLifecycleState?: string | null,
): string {
  const counts = new Map<string, number>();
  for (const row of rows) {
    const status = deriveAdvisoryState(
      advisoryInputFromMirrorRow(row, sessionLifecycleState),
    );
    const key = status.terminal
      ? "committed"
      : status.blocked
        ? "blocked"
        : status.advisory_state ?? "pending";
    counts.set(key, (counts.get(key) ?? 0) + 1);
  }
  if (counts.size === 0) return "";
  return [...counts.entries()]
    .map(([state, n]) => `${n} ${state.replace(/_/g, " ")}`)
    .join(" · ");
}

export function deriveAdvisoryForRow(
  row: CaptureHandoffRow,
  sessionLifecycleState?: string | null,
  options?: { poseAttested?: boolean },
) {
  return deriveAdvisoryState(
    advisoryInputFromMirrorRow(row, sessionLifecycleState, options),
  );
}

export function formatAdvisoryStateKey(state: AdvisoryState | null, terminal?: string): string {
  if (terminal) return "committed";
  return state ?? "pending";
}

export function enrichAdvisoryRow(
  row: CaptureHandoffRow,
  sessionLifecycleState?: string | null,
  options?: { experimentWarn?: boolean; poseAttested?: boolean },
): EnrichedAdvisoryRow {
  const status = deriveAdvisoryForRow(row, sessionLifecycleState, {
    poseAttested: options?.poseAttested,
  });
  const workflowPhase = row.workflow_phase;
  return {
    capture_candidate_id: row.capture_candidate_id,
    status,
    queue_priority: computeQueuePriority(status),
    blocker_groups: classifyBlockerGroups(status, {
      workflowPhase,
      experimentWarn: options?.experimentWarn,
    }),
    readiness_cohort: readinessCohort(status),
  };
}

export function buildSessionAdvisorySummary(
  rows: CaptureHandoffRow[],
  sessionLifecycleState?: string | null,
): SessionAdvisorySummary | null {
  if (!rows.length) return null;
  const enriched = rows.map((r) => enrichAdvisoryRow(r, sessionLifecycleState));
  const readiness_cohorts: Partial<Record<ReadinessCohortId, number>> = {};
  const blocker_groups: Partial<Record<BlockerGroupId, number>> = {};
  for (const e of enriched) {
    readiness_cohorts[e.readiness_cohort] =
      (readiness_cohorts[e.readiness_cohort] ?? 0) + 1;
    for (const g of e.blocker_groups) {
      blocker_groups[g] = (blocker_groups[g] ?? 0) + 1;
    }
  }
  return {
    total: rows.length,
    readiness_cohorts,
    blocker_groups,
  };
}

export function enrichAndSortRows(
  rows: CaptureHandoffRow[],
  sessionLifecycleState?: string | null,
): EnrichedAdvisoryRow[] {
  return sortEnrichedByQueue(
    rows.map((r) => enrichAdvisoryRow(r, sessionLifecycleState)),
  );
}

export function enrichRowsForTriage(
  rows: CaptureHandoffRow[],
  sessionLifecycleState?: string | null,
  options?: {
    experimentWarnCaptureIds?: ReadonlySet<string>;
    focusIds?: ReadonlySet<string>;
  },
): EnrichedAdvisoryRow[] {
  const warnSet = options?.experimentWarnCaptureIds;
  const focusIds = options?.focusIds;
  const enriched = rows.map((r) =>
    enrichAdvisoryRowV2(r, sessionLifecycleState, {
      experimentWarn: warnSet?.has(r.capture_candidate_id) ?? false,
      experimentWarnIds: warnSet,
      inFocusSet: focusIds?.has(r.capture_candidate_id) ?? false,
    }),
  );
  return sortEnrichedByQueue(enriched);
}
