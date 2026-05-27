import { readFileSync } from "node:fs";
import { join } from "node:path";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { FORBIDDEN_LEXICON } from "@/governance/banners";
import { deriveExperimentAnalytics } from "./analyticsDerive";
import { ExperimentRepeatabilityTrendStrip } from "./ExperimentRepeatabilityTrendStrip";
import { experimentManifestSchema } from "./experimentSchema";
import { computeSpecFingerprint, parseExperimentSpec } from "./experimentSpecCompile";
import { deriveExperimentMetrics } from "./metricsDerive";

describe("ExperimentRepeatabilityTrendStrip", () => {
  it("renders repeatability trend without forbidden lexicon", () => {
    const spec = parseExperimentSpec(
      JSON.parse(
        readFileSync(
          join(
            import.meta.dirname,
            "../../../../fixtures/rt_experiments/f5_spec_examples/repeatability_sweep.json",
          ),
          "utf8",
        ),
      ),
    );
    const fp = computeSpecFingerprint(spec);
    const manifest = experimentManifestSchema.parse({
      schema: "rt_experiment_manifest_v1",
      experiment_id: "exp-repeat-test",
      created_at_utc: "2026-05-26T15:00:00+00:00",
      governance_banner: "RT EXPERIMENT — explanatory compare only; not operational authority",
      runs: [
        {
          run_id: "r0",
          label: "run 0",
          session_id: "s0",
          recorded_at_utc: "2026-05-26T15:00:00+00:00",
          experiment_class: "repeatability_sweep",
          spec_fingerprint: fp,
          repeat_index: 0,
          repeat_group_id: "grp",
          snapshot: { world_summary: { entity_count: 2 }, tactical_state: { tactical_mode: "manual" } },
        },
        {
          run_id: "r1",
          label: "run 1",
          session_id: "s1",
          recorded_at_utc: "2026-05-26T15:01:00+00:00",
          experiment_class: "repeatability_sweep",
          spec_fingerprint: fp,
          repeat_index: 1,
          repeat_group_id: "grp",
          snapshot: {
            world_summary: { entity_count: 3 },
            tactical_state: { tactical_mode: "assisted", tti_s: 2.0 },
          },
          visibility_context: { los_cognition_label: "clear", occlusion_marker_count: 1 },
        },
      ],
    });
    const f1 = deriveExperimentAnalytics(manifest);
    const metricsReport = deriveExperimentMetrics(manifest, f1, { spec });
    const markup = renderToStaticMarkup(
      <ExperimentRepeatabilityTrendStrip
        manifest={manifest}
        metricsReport={metricsReport}
        f1PerRun={f1.per_run}
      />,
    );
    const text = markup.toLowerCase();
    for (const term of FORBIDDEN_LEXICON) {
      expect(text).not.toMatch(new RegExp(`\\b${term}\\b`, "i"));
    }
    expect(markup).toContain('data-testid="experiment-repeatability-trend-strip"');
    expect(text).not.toContain("winner");
  });
});
