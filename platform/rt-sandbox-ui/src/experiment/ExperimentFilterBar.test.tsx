import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { FORBIDDEN_LEXICON } from "@/governance/banners";
import { ExperimentFilterBar } from "./ExperimentFilterBar";
import { EMPTY_F5_FILTERS } from "./experimentF5UiHelpers";
import { loadF5GoldenMetrics } from "./f5TestFixtures";
import { collectFilterOptions } from "./experimentF5UiHelpers";
import { experimentManifestSchema } from "./experimentSchema";
import { readFileSync } from "node:fs";
import { join } from "node:path";

describe("ExperimentFilterBar", () => {
  it("renders filter bar without forbidden lexicon", () => {
    const { metrics } = loadF5GoldenMetrics();
    const manifest = experimentManifestSchema.parse(
      JSON.parse(
        readFileSync(
          join(import.meta.dirname, "../../../../fixtures/rt_experiments/f5_metrics_golden/manifest.json"),
          "utf8",
        ),
      ),
    );
    const options = collectFilterOptions(manifest.runs, metrics.per_run_extended);
    const markup = renderToStaticMarkup(
      <ExperimentFilterBar
        filters={EMPTY_F5_FILTERS}
        onChange={() => {}}
        options={options}
      />,
    );
    const text = markup.toLowerCase();
    for (const term of FORBIDDEN_LEXICON) {
      expect(text).not.toMatch(new RegExp(`\\b${term}\\b`, "i"));
    }
    expect(markup).toContain('data-testid="experiment-filter-bar"');
  });
});
