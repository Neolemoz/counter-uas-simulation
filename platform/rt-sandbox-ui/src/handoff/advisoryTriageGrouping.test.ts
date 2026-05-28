import { describe, expect, it } from "vitest";
import { enrichAdvisoryRow } from "./advisoryAggregate";
import type { CaptureHandoffRow } from "@/bridge/types";
import { enrichAdvisoryRowV2 } from "./advisoryAggregationV2";
import {
  groupEnrichedRows,
  handoffStageKeyForRow,
  primaryBlockerGroup,
  rollupBlockerGroupExemplars,
} from "./advisoryTriageGrouping";

function mirrorRow(
  id: string,
  overrides: Partial<CaptureHandoffRow> = {},
): CaptureHandoffRow {
  return {
    schema: "rt_capture_handoff_row_v1",
    capture_candidate_id: id,
    approval_status: "pending",
    normalization_status: "normalized",
    workflow_phase: "review_pending",
    validation_ok: true,
    has_handoff_manifest: false,
    has_import_record: false,
    source_origin: "rt_sandbox_capture_v1",
    lineage_note: "",
    governance_banner: "",
    ...overrides,
  };
}

describe("advisoryTriageGrouping", () => {
  it("orders queue_band groups P3 before P6", () => {
    const review = enrichAdvisoryRow(
      mirrorRow("cap-review", { workflow_phase: "ready", approval_status: "pending" }),
    );
    const importReady = enrichAdvisoryRow(
      mirrorRow("cap-import", {
        workflow_phase: "prepared",
        approval_status: "approved",
        has_handoff_manifest: true,
        last_export_event_type: "handoff_prepared",
      }),
    );

    const groups = groupEnrichedRows([importReady, review], "queue_band");
    const keys = groups.map((g) => g.key);
    expect(keys.indexOf("P3_review")).toBeLessThan(keys.indexOf("P6_import"));
  });

  it("places capture in multiple blocker groups", () => {
    const row = enrichAdvisoryRow(
      mirrorRow("cap-1", {
        validation_ok: false,
        validation_errors: ["normalization_invalid"],
      }),
    );
    const groups = groupEnrichedRows([row], "blocker");
    expect(groups.length).toBeGreaterThan(0);
  });

  it("groups experiment warn captures", () => {
    const row = enrichAdvisoryRow(mirrorRow("cap-warn"), null, { experimentWarn: true });
    const groups = groupEnrichedRows([row], "experiment", {
      experimentRollup: {
        handoff_eligibility: "partial",
        warn_capture_ids: ["cap-warn"],
      },
    });
    expect(groups).toHaveLength(1);
    expect(groups[0]?.rows[0]?.capture_candidate_id).toBe("cap-warn");
  });

  it("primaryBlockerGroup follows taxonomy order", () => {
    expect(
      primaryBlockerGroup(["packaging", "normalization", "lineage"]),
    ).toBe("normalization");
  });

  it("groups cohort_v2 by readiness_cohort_v2", () => {
    const row = enrichAdvisoryRowV2(mirrorRow("cap-mb"), null, {
      experimentWarn: false,
    });
    const multi = {
      ...row,
      blocker_groups: ["normalization", "approval_gate"] as typeof row.blocker_groups,
      readiness_cohort_v2: "multi_blocker" as const,
    };
    const groups = groupEnrichedRows([multi], "cohort_v2");
    expect(groups.some((g) => g.key === "multi_blocker")).toBe(true);
  });

  it("groups handoff_stage by lane", () => {
    const approve = enrichAdvisoryRow(
      mirrorRow("cap-r", { workflow_phase: "ready", approval_status: "pending" }),
    );
    expect(handoffStageKeyForRow(approve)).toBe("approve");
    const groups = groupEnrichedRows([approve], "handoff_stage");
    expect(groups[0]?.key).toBe("approve");
  });

  it("rollupBlockerGroupExemplars caps exemplars at 5", () => {
    const rows = Array.from({ length: 8 }, (_, i) =>
      enrichAdvisoryRow(mirrorRow(`cap-${i}`), null, {
        experimentWarn: true,
      }),
    );
    const rollup = rollupBlockerGroupExemplars(rows, 5);
    expect(rollup.experiment_warn?.exemplar_capture_ids.length).toBeLessThanOrEqual(5);
    expect(rollup.experiment_warn?.count).toBe(8);
  });
});
