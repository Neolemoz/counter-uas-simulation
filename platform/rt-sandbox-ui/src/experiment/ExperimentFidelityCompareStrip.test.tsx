import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { containsForbiddenLexicon } from "@/cesium/cognition";
import { FIDELITY_METRICS_GOVERNANCE_BANNER } from "./experimentSchema";
import { ExperimentFidelityCompareStrip } from "./ExperimentFidelityCompareStrip";
import { deriveF5bGoldenFidelityMetrics, loadF5bGoldenManifest } from "./f5bTestFixtures";

describe("ExperimentFidelityCompareStrip", () => {
  it("renders fidelity compare badges for golden report", () => {
    const manifest = loadF5bGoldenManifest();
    const report = deriveF5bGoldenFidelityMetrics();
    const markup = renderToStaticMarkup(
      <ExperimentFidelityCompareStrip manifest={manifest} fidelityReport={report} />,
    );
    expect(markup).toContain('data-testid="experiment-fidelity-compare-strip"');
    expect(markup).toContain("truth_attested");
    expect(markup).toContain("explanatory");
    expect(markup).toContain("cognition_truth_divergence");
    expect(markup).toContain("pose_truth_drift_delta");
    expect(markup).toContain(FIDELITY_METRICS_GOVERNANCE_BANNER);
    expect(containsForbiddenLexicon(FIDELITY_METRICS_GOVERNANCE_BANNER)).toBe(false);
  });
});
