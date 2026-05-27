import { existsSync, readFileSync } from "node:fs";
import { join, resolve } from "node:path";
import { experimentManifestSchema, type ExperimentFidelityMetricsReport } from "./experimentSchema";
import { deriveExperimentFidelityMetrics } from "./fidelityMetricsDerive";
import type { StagingReader } from "./metricsDerive";

const F5B_DIR = join(
  import.meta.dirname,
  "../../../../fixtures/rt_experiments/f5b_fidelity_examples",
);

export function repoStagingReader(repoRoot: string): StagingReader {
  const root = resolve(repoRoot);
  return (ref: string) => {
    const path = resolve(root, ref);
    if (!existsSync(path)) return null;
    try {
      return readFileSync(path, "utf8");
    } catch {
      return null;
    }
  };
}

export function loadF5bGoldenManifest() {
  const raw = JSON.parse(
    readFileSync(join(F5B_DIR, "manifest_fidelity_golden.json"), "utf8"),
  );
  return experimentManifestSchema.parse(raw);
}

export function loadF5bGoldenFidelityMetrics(): ExperimentFidelityMetricsReport {
  return JSON.parse(
    readFileSync(join(F5B_DIR, "fidelity_metrics_report_golden.json"), "utf8"),
  );
}

export function deriveF5bGoldenFidelityMetrics(
  repoRoot = join(import.meta.dirname, "../../../.."),
): ExperimentFidelityMetricsReport {
  const manifest = loadF5bGoldenManifest();
  return deriveExperimentFidelityMetrics(manifest, undefined, {
    stagingReader: repoStagingReader(repoRoot),
  });
}
