import { describe, expect, it } from "vitest";
import { enrichAdvisoryRow } from "./advisoryAggregate";
import type { CaptureHandoffRow } from "@/bridge/types";
import {
  buildClientExportPreviewV2,
  formatExportJsonPreview,
} from "./advisoryBatchExportPreview";
import { containsForbiddenLexicon } from "@/cesium/cognition";

function mirrorRow(id: string): CaptureHandoffRow {
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
  };
}

describe("advisoryBatchExportPreview", () => {
  it("builds v2-shaped client preview", () => {
    const rows = [enrichAdvisoryRow(mirrorRow("cap-a"))];
    const preview = buildClientExportPreviewV2(rows);
    expect(preview.schema).toBe("rt_advisory_batch_review_v2");
    expect(preview.dry_run).toBe(true);
    expect(preview.summary.total).toBe(1);
    expect(preview.grouped.by_readiness_cohort).toBeDefined();
    expect(preview.standup.warn_only_notes.length).toBeGreaterThan(0);
    expect(preview.rollups_v2).toBeDefined();
    const json = formatExportJsonPreview(preview);
    expect(json).toContain("rt_advisory_batch_review_v2");
    expect(json).not.toContain("readiness_score");
    expect(containsForbiddenLexicon(preview.governance_banner)).toBe(false);
  });
});
