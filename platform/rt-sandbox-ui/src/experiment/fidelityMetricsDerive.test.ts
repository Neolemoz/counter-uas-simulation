import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import {
  deriveF5bGoldenFidelityMetrics,
  loadF5bGoldenFidelityMetrics,
  loadF5bGoldenManifest,
  repoStagingReader,
} from "./f5bTestFixtures";
import {
  buildFidelityComparePairs,
  deriveExperimentFidelityMetrics,
  exportFidelityMetricsJson,
  FORBIDDEN_FIDELITY_ROLLUP_KEYS,
  perRunFidelityFromRun,
} from "./fidelityMetricsDerive";

const F5B_DIR = join(
  import.meta.dirname,
  "../../../../fixtures/rt_experiments/f5b_fidelity_examples",
);

const REPO_ROOT = join(import.meta.dirname, "../../../..");

describe("fidelityMetricsDerive", () => {
  it("derives deterministic fidelity report matching golden", () => {
    const manifest = loadF5bGoldenManifest();
    const reader = repoStagingReader(REPO_ROOT);
    const r1 = deriveExperimentFidelityMetrics(manifest, undefined, { stagingReader: reader });
    const r2 = deriveExperimentFidelityMetrics(manifest, undefined, { stagingReader: reader });
    expect(r1).toEqual(r2);
    expect(r1.schema).toBe("rt_experiment_fidelity_metrics_report_v1");
    expect(r1.coupling_required).toBe(true);

    const golden = loadF5bGoldenFidelityMetrics();
    expect(r1).toEqual(golden);
  });

  it("flags cognition_truth_divergence on run-divergent", () => {
    const manifest = loadF5bGoldenManifest();
    const reader = repoStagingReader(REPO_ROOT);
    const divergent = perRunFidelityFromRun(manifest.runs[1], reader);
    expect(divergent.cognition_truth_divergence).toBe(true);
    expect(divergent.los_truth_label).toBe("terrain_blocked");
    expect(divergent.los_cognition_label).toBe("partial");
  });

  it("builds compare pair badges for golden runs", () => {
    const report = deriveF5bGoldenFidelityMetrics();
    const pair = report.compare_pairs_fidelity.find(
      (p) => p.run_id_a === "run-clear" && p.run_id_b === "run-divergent",
    );
    expect(pair).toBeTruthy();
    expect(pair!.badges.map((b) => b.id)).toEqual([
      "cognition_truth_divergence",
      "los_truth_label_diff",
      "pose_truth_drift_delta",
    ]);
  });

  it("rollup excludes forbidden keys", () => {
    const report = deriveF5bGoldenFidelityMetrics();
    const rollupJson = JSON.stringify(report.rollup_fidelity);
    for (const key of FORBIDDEN_FIDELITY_ROLLUP_KEYS) {
      expect(rollupJson).not.toContain(`"${key}"`);
    }
  });

  it("export adds derived_at_utc", () => {
    const report = deriveF5bGoldenFidelityMetrics();
    const exported = JSON.parse(exportFidelityMetricsJson(report));
    expect(exported.derived_at_utc).toBeTruthy();
  });

  it("returns unavailable rollup when coupling off and no staging", () => {
    const manifest = loadF5bGoldenManifest();
    const noCoupling = {
      ...manifest,
      runs: manifest.runs.map((r) => ({
        ...r,
        fidelity_context: undefined,
      })),
    };
    const report = deriveExperimentFidelityMetrics(noCoupling);
    expect(report.coupling_required).toBe(false);
    expect(report.per_run_fidelity.every((r) => r.fidelity_attestation_status === "unavailable")).toBe(
      true,
    );
  });
});

describe("fidelityMetricsDerive golden file sync", () => {
  it("golden fixture matches derive output (update golden if intentional)", () => {
    const derived = deriveF5bGoldenFidelityMetrics();
    const goldenPath = join(F5B_DIR, "fidelity_metrics_report_golden.json");
    const golden = JSON.parse(readFileSync(goldenPath, "utf8"));
    if (JSON.stringify(derived) !== JSON.stringify(golden)) {
      // eslint-disable-next-line no-console
      console.log("Computed golden diff — update fidelity_metrics_report_golden.json");
      console.log(JSON.stringify(derived, null, 2));
    }
    expect(derived).toEqual(golden);
  });
});
