import { describe, expect, it } from "vitest";
import { enrichRowsForTriage } from "./advisoryAggregate";
import {
  applyFilterPreset,
  buildSessionAdvisorySummaryV2,
  countRowsForStandupPass,
  readinessCohortV2,
  rollupHandoff,
  STANDUP_PASSES,
} from "./advisoryAggregationV2";
import type { EnrichedAdvisoryRow } from "./advisoryTypes";
import type { AdvisoryStatus } from "./advisoryTypes";

function mockRow(
  id: string,
  state: AdvisoryStatus["advisory_state"],
  groups: EnrichedAdvisoryRow["blocker_groups"] = [],
): EnrichedAdvisoryRow {
  const status: AdvisoryStatus = {
    schema: "rt_sa_workflow_advisory_status_v1",
    capture_candidate_id: id,
    advisory_state: state,
    advisory_state_label: state ?? "pending",
    blocked: false,
    block_reasons: [],
    upstream: {},
    governance_banner: "SA WORKFLOW ADVISORY — explanatory; maintainer CLIs are authority",
  };
  return {
    capture_candidate_id: id,
    status,
    queue_priority: { rank: 100, band: "P3_review", rationale: "test" },
    blocker_groups: groups,
    readiness_cohort: "needs_review",
  };
}

describe("advisoryAggregationV2", () => {
  it("filters import_advisory_only preset", () => {
    const rows = [
      mockRow("a", "import_ready"),
      mockRow("b", "capture_ready"),
    ];
    const filtered = applyFilterPreset(rows, "import_advisory_only");
    expect(filtered.map((r) => r.capture_candidate_id)).toEqual(["a"]);
  });

  it("rollupHandoff counts stages", () => {
    const hr = rollupHandoff([
      { ...mockRow("a", "capture_ready"), readiness_cohort: "needs_review" },
      {
        ...mockRow("b", "import_ready"),
        readiness_cohort: "ready_for_commit_advisory",
      },
    ]);
    expect(hr.by_stage.review).toBe(1);
    expect(hr.by_stage.import_advisory).toBe(1);
  });

  it("readinessCohortV2 multi_blocker", () => {
    const row = mockRow("x", "approval_ready", ["normalization", "approval_gate"]);
    const c2 = readinessCohortV2(row);
    expect(c2).toBe("multi_blocker");
  });

  it("buildSessionAdvisorySummaryV2 sets schema_version f8", () => {
    const summary = buildSessionAdvisorySummaryV2([], "active");
    expect(summary).toBeNull();
  });

  it("enrichRowsForTriage sets readiness_cohort_v2", () => {
    const rows = enrichRowsForTriage([
      {
        capture_candidate_id: "t1",
        schema: "rt_capture_handoff_row_v1",
        approval_status: "pending",
        normalization_status: "normalized",
        workflow_phase: "review_pending",
        validation_ok: true,
        has_handoff_manifest: false,
        has_import_record: false,
        source_origin: "rt_sandbox_capture_v1",
        lineage_note: "",
        governance_banner: "",
      },
    ]);
    expect(rows[0]?.readiness_cohort_v2).toBeTruthy();
    expect(JSON.stringify(rows)).not.toContain("readiness_score");
  });

  it("countRowsForStandupPass respects import_advisory preset", () => {
    const rows = [
      mockRow("a", "import_ready"),
      mockRow("b", "capture_ready"),
    ];
    const passE = STANDUP_PASSES.find((p) => p.pass_id === "pass_e_import_advisory")!;
    expect(countRowsForStandupPass(rows, passE)).toBe(1);
  });
});
