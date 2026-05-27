import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ADVISORY_FORBIDDEN_LEXICON } from "@/governance/banners";
import { deriveAdvisoryState } from "@/handoff/deriveAdvisoryState";
import type { AdvisoryDeriveInput } from "@/handoff/advisoryTypes";
import { ExperimentImportAdvisoryStrip } from "./ExperimentImportAdvisoryStrip";
import { loadF5GoldenMetrics } from "./f5TestFixtures";

describe("ExperimentImportAdvisoryStrip", () => {
  it("renders import advisory without forbidden lexicon or submit", () => {
    const input: AdvisoryDeriveInput = {
      capture_candidate_id: "cap-x",
      candidate: { normalization_status: "normalized", approval_status: "approved" },
      export_events: [
        "capture_approved",
        "conversion_manifest_written",
        "handoff_import_prepared",
      ],
      handoff_manifest_present: true,
      conversion_steps_advisory_pass: true,
      origin: "rt_sandbox_capture_v1",
      session_lifecycle_state: "running",
    };
    const status = deriveAdvisoryState(input);
    const { metrics } = loadF5GoldenMetrics();
    const markup = renderToStaticMarkup(
      <ExperimentImportAdvisoryStrip
        advisoryStatus={status}
        handoff={metrics.handoff_eligibility}
      />,
    );
    const text = markup.toLowerCase();
    for (const term of ADVISORY_FORBIDDEN_LEXICON) {
      expect(text).not.toMatch(new RegExp(`\\b${term}\\b`, "i"));
    }
    expect(markup).toContain('data-testid="experiment-import-advisory-strip"');
    expect(markup).toContain("rt_sa_import commit");
    expect(markup).not.toMatch(/type="submit"/);
  });
});
