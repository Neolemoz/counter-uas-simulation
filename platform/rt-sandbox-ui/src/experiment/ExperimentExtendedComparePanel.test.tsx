import { readFileSync } from "node:fs";
import { join } from "node:path";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { FORBIDDEN_LEXICON } from "@/governance/banners";
import { ExperimentExtendedComparePanel } from "./ExperimentExtendedComparePanel";
import { experimentManifestSchema } from "./experimentSchema";
import { loadF5GoldenMetrics } from "./f5TestFixtures";
import { computeSpecFingerprint, parseExperimentSpec } from "./experimentSpecCompile";

describe("ExperimentExtendedComparePanel", () => {
  it("renders extended compare without forbidden lexicon", () => {
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
    const ids = manifest.runs.slice(0, 2).map((r) => r.run_id);
    const markup = renderToStaticMarkup(
      <ExperimentExtendedComparePanel
        filteredRuns={manifest.runs}
        perRunExtended={metrics.per_run_extended}
        metricsReport={metrics}
        selectedRunIds={ids}
        onSelectedRunIdsChange={() => {}}
      />,
    );
    const text = markup.toLowerCase();
    for (const term of FORBIDDEN_LEXICON) {
      expect(text).not.toMatch(new RegExp(`\\b${term}\\b`, "i"));
    }
    expect(markup).toContain('data-testid="experiment-extended-compare-panel"');
    expect(text).not.toContain("winner");
  });
});
