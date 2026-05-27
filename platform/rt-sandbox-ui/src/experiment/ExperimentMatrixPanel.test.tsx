import { readFileSync } from "node:fs";
import { join } from "node:path";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { FORBIDDEN_LEXICON } from "@/governance/banners";
import { ExperimentMatrixPanel } from "./ExperimentMatrixPanel";
import { experimentManifestSchema } from "./experimentSchema";
import { loadF5GoldenMetrics } from "./f5TestFixtures";
import { computeSpecFingerprint, parseExperimentSpec } from "./experimentSpecCompile";

describe("ExperimentMatrixPanel", () => {
  it("renders matrix panel for parameter_matrix class", () => {
    const spec = parseExperimentSpec(
      JSON.parse(
        readFileSync(
          join(
            import.meta.dirname,
            "../../../../fixtures/rt_experiments/f5_spec_examples/parameter_matrix.json",
          ),
          "utf8",
        ),
      ),
    );
    const fp = computeSpecFingerprint(spec);
    const manifestRaw = JSON.parse(
      readFileSync(
        join(import.meta.dirname, "../../../../fixtures/rt_experiments/f5_metrics_golden/manifest.json"),
        "utf8",
      ),
    );
    for (const run of manifestRaw.runs) {
      run.spec_fingerprint = fp;
    }
    const manifest = experimentManifestSchema.parse(manifestRaw);
    const { metrics } = loadF5GoldenMetrics();
    const markup = renderToStaticMarkup(
      <ExperimentMatrixPanel
        manifest={manifest}
        metricsReport={metrics}
        axisRow="template_id"
        axisCol="dwell_s"
        onAxisRowChange={() => {}}
        onAxisColChange={() => {}}
      />,
    );
    const text = markup.toLowerCase();
    for (const term of FORBIDDEN_LEXICON) {
      expect(text).not.toMatch(new RegExp(`\\b${term}\\b`, "i"));
    }
    expect(markup).toContain('data-testid="experiment-matrix-panel"');
    expect(markup).toContain("matrix rollup");
  });
});
