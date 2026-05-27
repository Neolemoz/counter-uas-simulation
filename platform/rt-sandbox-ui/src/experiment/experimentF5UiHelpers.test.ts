import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { deriveExperimentAnalytics } from "./analyticsDerive";
import { experimentManifestSchema } from "./experimentSchema";
import { computeSpecFingerprint, parseExperimentSpec } from "./experimentSpecCompile";
import {
  buildMatrixGrid,
  collectMatrixAxisKeys,
  EMPTY_F5_FILTERS,
  filterManifestRuns,
  F5_FILTER_ALL,
  handoffDisplayLevel,
  pairsForRuns,
} from "./experimentF5UiHelpers";
import { deriveExperimentMetrics } from "./metricsDerive";

const GOLDEN_DIR = join(
  import.meta.dirname,
  "../../../../fixtures/rt_experiments/f5_metrics_golden",
);

function loadGoldenMetrics() {
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
  const manifestRaw = JSON.parse(readFileSync(join(GOLDEN_DIR, "manifest.json"), "utf8"));
  for (const run of manifestRaw.runs) {
    run.spec_fingerprint = fp;
  }
  const manifest = experimentManifestSchema.parse(manifestRaw);
  const f1 = deriveExperimentAnalytics(manifest);
  const metrics = deriveExperimentMetrics(manifest, f1, { spec });
  return { manifest, metrics, spec };
}

describe("experimentF5UiHelpers", () => {
  it("maps handoff levels to display vocabulary", () => {
    expect(handoffDisplayLevel("eligible")).toBe("eligible");
    expect(handoffDisplayLevel("partial")).toBe("review_needed");
    expect(handoffDisplayLevel("ineligible")).toBe("blocked");
  });

  it("filters runs by experiment class and tactical mode", () => {
    const { manifest, metrics } = loadGoldenMetrics();
    const filtered = filterManifestRuns(
      manifest.runs,
      {
        ...EMPTY_F5_FILTERS,
        experiment_class: "parameter_matrix",
        tactical_mode: "manual",
      },
      metrics.per_run_extended,
    );
    expect(filtered.length).toBeGreaterThan(0);
    expect(filtered.every((r) => r.experiment_class === "parameter_matrix")).toBe(true);
  });

  it("builds matrix grid from matrix_coords", () => {
    const { manifest } = loadGoldenMetrics();
    const keys = collectMatrixAxisKeys(manifest.runs);
    expect(keys).toContain("template_id");
    expect(keys).toContain("dwell_s");
    const grid = buildMatrixGrid(manifest.runs, "template_id", "dwell_s");
    expect(grid.rows.length).toBeGreaterThan(0);
    expect(grid.cols.length).toBeGreaterThan(0);
    const populated = grid.cells.filter((c) => c.runId !== null);
    expect(populated.length).toBe(4);
  });

  it("subsets compare pairs for selected runs", () => {
    const { manifest, metrics } = loadGoldenMetrics();
    const ids = manifest.runs.slice(0, 2).map((r) => r.run_id);
    const pairs = pairsForRuns(metrics.compare_pairs_extended, ids);
    expect(pairs.length).toBe(1);
    expect(pairs[0].run_id_a).toBe(ids[0]);
    expect(pairs[0].run_id_b).toBe(ids[1]);
  });

  it("F5_FILTER_ALL passes all runs", () => {
    const { manifest, metrics } = loadGoldenMetrics();
    expect(
      filterManifestRuns(manifest.runs, EMPTY_F5_FILTERS, metrics.per_run_extended).length,
    ).toBe(manifest.runs.length);
    expect(F5_FILTER_ALL).toBe("__all__");
  });
});
