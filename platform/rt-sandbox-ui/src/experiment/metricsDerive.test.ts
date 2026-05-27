import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { deriveExperimentAnalytics } from "./analyticsDerive";
import { experimentManifestSchema } from "./experimentSchema";
import { computeSpecFingerprint, parseExperimentSpec } from "./experimentSpecCompile";
import { deriveExperimentMetrics, exportMetricsJson } from "./metricsDerive";

const GOLDEN_DIR = join(
  import.meta.dirname,
  "../../../../fixtures/rt_experiments/f5_metrics_golden",
);

describe("metricsDerive", () => {
  it("derives deterministic metrics report for matrix golden", () => {
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
    const m1 = deriveExperimentMetrics(manifest, f1, { spec });
    const m2 = deriveExperimentMetrics(manifest, f1, { spec });
    expect(m1).toEqual(m2);
    expect(m1.schema).toBe("rt_experiment_metrics_report_v1");
    expect(m1.per_run_extended).toHaveLength(4);
    expect(m1.rollup_extended.matrix_rollup.expected_cells).toBe(4);
    expect(m1.rollup_extended.terrain_rollup.ridge_counts.north_ridge).toBe(2);
    expect(m1.compare_pairs_extended.length).toBeGreaterThan(0);
    const terrainBadge = m1.compare_pairs_extended.find((p) =>
      p.badges.some((b) => b.id === "terrain_context_diff"),
    );
    expect(terrainBadge).toBeTruthy();
  });

  it("handoff ineligible when captures missing", () => {
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
    const manifestRaw = JSON.parse(readFileSync(join(GOLDEN_DIR, "manifest.json"), "utf8"));
    for (const run of manifestRaw.runs) {
      delete run.capture_candidate_id;
      delete run.capture_staging_ref;
    }
    const manifest = experimentManifestSchema.parse(manifestRaw);
    const f1 = deriveExperimentAnalytics(manifest);
    const metrics = deriveExperimentMetrics(manifest, f1, { spec });
    expect(metrics.handoff_eligibility.experiment_level).toBe("ineligible");
    expect(
      metrics.handoff_eligibility.gates.find((g) => g.id === "all_captures_present")?.pass,
    ).toBe(false);
  });

  it("handoff eligible when gates pass", () => {
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
      run.capture_candidate_id = `cap-${run.run_id}`;
      run.capture_staging_ref = `runs/rt_sandbox/captures/cap-${run.run_id}`;
    }
    const manifest = experimentManifestSchema.parse(manifestRaw);
    const f1 = deriveExperimentAnalytics(manifest);
    for (const row of f1.per_run) {
      row.normalization_status_ref = "normalized";
    }
    const metrics = deriveExperimentMetrics(manifest, f1, {
      spec,
      maintainerAckPoseReviewed: true,
    });
    expect(metrics.handoff_eligibility.experiment_level).toBe("eligible");
  });

  it("export adds derived_at_utc when requested", () => {
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
    const manifest = experimentManifestSchema.parse(
      JSON.parse(readFileSync(join(GOLDEN_DIR, "manifest.json"), "utf8")),
    );
    const f1 = deriveExperimentAnalytics(manifest);
    const metrics = deriveExperimentMetrics(manifest, f1, { spec });
    const text = exportMetricsJson(metrics, { derived_at_utc: "2026-05-26T15:00:00+00:00" });
    expect(text).toContain("derived_at_utc");
  });
});
