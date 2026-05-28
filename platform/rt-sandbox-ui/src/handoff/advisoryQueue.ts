import type {
  AdvisoryStatus,
  BlockerGroupId,
  QueueBandId,
  QueuePriority,
  ReadinessCohortId,
} from "./advisoryTypes";

const BLOCKER_GROUP_ORDER: BlockerGroupId[] = [
  "normalization",
  "review_attestation",
  "approval_gate",
  "packaging",
  "lineage",
  "experiment_warn",
  "terminal_block",
];

export function detectLineageWarnings(candidate?: Record<string, unknown> | null): string[] {
  const warnings: string[] = [];
  if (!candidate) return warnings;
  const parent = candidate.parent_ref;
  const sessionId = candidate.session_id;
  if (typeof parent === "string" && typeof sessionId === "string" && parent === sessionId) {
    warnings.push("LIN-01: parent_ref resembles session_id — not SA lineage authority");
  }
  const origin = candidate.origin;
  const originStr = Array.isArray(origin) ? origin.join(" ") : String(origin ?? "");
  if (candidate.normalization_status === "normalized" && !originStr.includes("rt_sandbox_capture_v1")) {
    warnings.push("LIN-02: origin missing rt_sandbox_capture_v1");
  }
  if (candidate.authoritative_parent_ref) {
    warnings.push("LIN-03: authoritative_parent_ref forbidden on staging candidate");
  }
  return warnings;
}

export function classifyBlockerGroups(
  status: AdvisoryStatus,
  options?: { workflowPhase?: string | null; experimentWarn?: boolean },
): BlockerGroupId[] {
  const groups: BlockerGroupId[] = [];
  const reasons = status.block_reasons ?? [];

  if (status.blocked) {
    if (reasons.includes("handoff_rejected") || reasons.includes("handoff_import_deferred")) {
      groups.push("terminal_block");
    }
  }

  for (const item of status.checklist ?? []) {
    if (item.status === "fail") {
      if (item.id === "normalization" || item.id === "validation_doc") {
        groups.push("normalization");
      } else if (item.id === "pose_cognition" || item.id === "export_audit") {
        groups.push("review_attestation");
      } else if (item.id === "lineage" || item.id === "origin") {
        groups.push("lineage");
      }
    } else if (item.status === "warn" && item.id === "pose_cognition") {
      groups.push("review_attestation");
    }
  }

  if (status.advisory_state === "approval_ready") groups.push("approval_gate");
  if (status.advisory_state === "handoff_ready") groups.push("packaging");
  if (status.lineage_warnings?.length) groups.push("lineage");
  if (options?.experimentWarn) groups.push("experiment_warn");

  const seen = new Set<BlockerGroupId>();
  const ordered: BlockerGroupId[] = [];
  for (const g of BLOCKER_GROUP_ORDER) {
    if (groups.includes(g) && !seen.has(g)) {
      seen.add(g);
      ordered.push(g);
    }
  }
  return ordered;
}

export function readinessCohort(
  status: AdvisoryStatus,
  options?: { error?: boolean },
): ReadinessCohortId {
  if (options?.error) return "error";
  if (status.terminal) return "terminal";
  if (status.blocked) return "blocked";
  const map: Record<string, ReadinessCohortId> = {
    capture_ready: "needs_review",
    review_complete: "needs_review",
    approval_ready: "needs_approve",
    handoff_ready: "needs_prepare",
    import_ready: "ready_for_commit_advisory",
  };
  if (status.advisory_state && map[status.advisory_state]) {
    return map[status.advisory_state];
  }
  return "needs_normalize";
}

export function computeQueuePriority(
  status: AdvisoryStatus,
  options?: { error?: string },
): QueuePriority {
  if (options?.error) {
    return { rank: 100, band: "P1_error", rationale: options.error };
  }
  const reasons = status.block_reasons ?? [];
  if (status.blocked) {
    const sub = reasons.includes("handoff_rejected") ? 0 : 10;
    return {
      rank: sub,
      band: "P0_block",
      rationale: `blocked: ${reasons.slice(0, 2).join(", ")}`,
    };
  }
  if (status.terminal) {
    return { rank: 900, band: "P7_terminal", rationale: "terminal committed" };
  }
  const bandMap: Record<string, { band: QueueBandId; base: number }> = {
    capture_ready: { band: "P3_review", base: 320 },
    review_complete: { band: "P3_review", base: 340 },
    approval_ready: { band: "P4_approve", base: 410 },
    handoff_ready: { band: "P5_package", base: 520 },
    import_ready: { band: "P6_import", base: 610 },
  };
  const state = status.advisory_state;
  const entry = state ? bandMap[state] : { band: "P2_normalize" as QueueBandId, base: 250 };
  return {
    rank: entry.base,
    band: entry.band,
    rationale: state ?? "not_ready",
  };
}

export function sortEnrichedByQueue<T extends { queue_priority: QueuePriority }>(rows: T[]): T[] {
  return [...rows].sort((a, b) => a.queue_priority.rank - b.queue_priority.rank);
}

export function queueBandLabel(band: string): string {
  return band.replace(/^P\d+_/, "").replace(/_/g, " ");
}
