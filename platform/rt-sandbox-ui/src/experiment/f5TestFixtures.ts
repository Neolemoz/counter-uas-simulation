import { readFileSync } from "node:fs";
import { join } from "node:path";
import { deriveExperimentAnalytics } from "./analyticsDerive";
import { experimentManifestSchema, type ExperimentMetricsReport } from "./experimentSchema";
import { computeSpecFingerprint, parseExperimentSpec } from "./experimentSpecCompile";
import { deriveExperimentMetrics } from "./metricsDerive";

const GOLDEN_DIR = join(
  import.meta.dirname,
  "../../../../fixtures/rt_experiments/f5_metrics_golden",
);

export function loadF5GoldenMetrics(): {
  metrics: ExperimentMetricsReport;
  experimentId: string;
} {
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
  return { metrics, experimentId: manifest.experiment_id };
}
