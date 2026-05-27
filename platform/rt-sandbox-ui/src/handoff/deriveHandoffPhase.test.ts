import { describe, expect, it } from "vitest";
import type { CaptureHandoffRow } from "@/bridge/types";
import {
  handoffChipsForRow,
  highestWorkflowPhase,
  phaseLabel,
} from "@/handoff/deriveHandoffPhase";

function row(phase: CaptureHandoffRow["workflow_phase"]): CaptureHandoffRow {
  return {
    schema: "rt_capture_handoff_row_v1",
    capture_candidate_id: "test-id",
    approval_status: "pending",
    normalization_status: "normalized",
    workflow_phase: phase,
    validation_ok: true,
    has_handoff_manifest: false,
    has_import_record: false,
    source_origin: "rt_sandbox_capture_v1",
    lineage_note: "session ref non-authoritative",
    governance_banner: "HANDOFF MIRROR",
  };
}

describe("deriveHandoffPhase", () => {
  it("picks highest priority phase across rows", () => {
    expect(
      highestWorkflowPhase([row("staged"), row("ready"), row("normalized")]),
    ).toBe("ready");
    expect(highestWorkflowPhase([row("committed"), row("ready")])).toBe("committed");
  });

  it("returns none for empty list", () => {
    expect(highestWorkflowPhase([])).toBe("none");
  });

  it("builds chips for ready and rejected", () => {
    expect(handoffChipsForRow(row("ready")).some((c) => c.tone === "ok")).toBe(true);
    expect(handoffChipsForRow(row("rejected")).some((c) => c.label === "rejected")).toBe(
      true,
    );
  });

  it("formats phase labels", () => {
    expect(phaseLabel("review_pending")).toBe("review pending");
  });
});
