import { readFileSync } from "node:fs";
import { join } from "node:path";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { BANNER_SA_WORKFLOW_ADVISORY } from "@/governance/banners";
import { containsForbiddenLexicon } from "@/cesium/cognition";
import { ADVISORY_GOVERNANCE_BANNER } from "./advisoryTypes";
import { enrichAdvisoryRow, enrichRowsForTriage } from "./advisoryAggregate";
import { buildSessionAdvisorySummaryV2 } from "./advisoryAggregationV2";
import { AdvisoryTriageQueuePanel } from "./AdvisoryTriageQueuePanel";
import type { FilterPresetId } from "./advisoryTypes";
import type { CaptureHandoffRow } from "@/bridge/types";

const FIXTURE_DIR = join(
  import.meta.dirname,
  "../../../../fixtures/rt_handoff/f7_advisory_examples",
);

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

describe("AdvisoryTriageQueuePanel", () => {
  it("renders triage panel with governance banners and no forbidden lexicon", () => {
    const raw = JSON.parse(
      readFileSync(join(FIXTURE_DIR, "queue_priority_mixed.json"), "utf8"),
    ) as {
      inputs: { captures: Array<{ capture_candidate_id: string; advisory_state: string }> };
    };
    const rows = raw.inputs.captures.map((c) => {
      const row = mirrorRow(c.capture_candidate_id);
      if (c.advisory_state === "import_ready") {
        row.workflow_phase = "prepared";
        row.approval_status = "approved";
        row.has_handoff_manifest = true;
      }
      if (c.advisory_state === "approval_ready") {
        row.workflow_phase = "ready";
        row.approval_status = "pending";
      }
      return enrichAdvisoryRow(row);
    });

    const markup = renderToStaticMarkup(
      <AdvisoryTriageQueuePanel
        rows={rows}
        selectedCaptureId={null}
        onSelectCapture={() => {}}
      />,
    );
    expect(markup).toContain('data-testid="advisory-triage-queue-panel"');
    expect(markup).toContain("maintainer CLIs are authority");
    expect(containsForbiddenLexicon(BANNER_SA_WORKFLOW_ADVISORY)).toBe(false);
    expect(containsForbiddenLexicon(ADVISORY_GOVERNANCE_BANNER)).toBe(false);
  });

  it("renders F8 hub with v2 chip and template preview when summary provided", () => {
    const rows = [mirrorRow("f8-cap-1"), mirrorRow("f8-cap-2")];
    const enriched = enrichRowsForTriage(rows);
    const summary = buildSessionAdvisorySummaryV2(rows)!;
    const markup = renderToStaticMarkup(
      <AdvisoryTriageQueuePanel
        rows={enriched}
        selectedCaptureId={null}
        onSelectCapture={() => {}}
        filterPreset={"all_staged" as FilterPresetId}
        onFilterPresetChange={() => {}}
        focusSet={new Set()}
        captureIdsForFocus={["f8-cap-1", "f8-cap-2"]}
        onFocusToggle={() => {}}
        onFocusClear={() => {}}
        sessionSummary={summary}
      />,
    );
    expect(markup).toContain("F8 hub");
    expect(markup).toContain("Filter preset ≠ CLI invocation");
    expect(markup).toContain("f8-template-preview");
    expect(markup).not.toContain("readiness_score");
  });

  it("renders empty state without captures", () => {
    const markup = renderToStaticMarkup(
      <AdvisoryTriageQueuePanel
        rows={[]}
        selectedCaptureId={null}
        onSelectCapture={() => {}}
      />,
    );
    expect(markup).toContain("No staged captures");
  });
});
