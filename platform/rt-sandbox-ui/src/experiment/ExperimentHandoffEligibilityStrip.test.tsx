import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { FORBIDDEN_LEXICON } from "@/governance/banners";
import { ExperimentHandoffEligibilityStrip } from "./ExperimentHandoffEligibilityStrip";
import { loadF5GoldenMetrics } from "./f5TestFixtures";

describe("ExperimentHandoffEligibilityStrip", () => {
  it("renders handoff strip with display level", () => {
    const { metrics } = loadF5GoldenMetrics();
    const markup = renderToStaticMarkup(
      <ExperimentHandoffEligibilityStrip
        handoff={metrics.handoff_eligibility}
        maintainerAckPoseReviewed={false}
        onMaintainerAckPoseReviewedChange={() => {}}
      />,
    );
    const text = markup.toLowerCase();
    for (const term of FORBIDDEN_LEXICON) {
      expect(text).not.toMatch(new RegExp(`\\b${term}\\b`, "i"));
    }
    expect(markup).toContain('data-testid="experiment-handoff-eligibility-strip"');
    expect(markup).toContain('data-testid="handoff-display-level"');
    expect(markup).not.toMatch(/type="submit"/);
    expect(text).not.toContain("handoff_import");
  });
});
