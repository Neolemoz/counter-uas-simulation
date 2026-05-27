import { describe, expect, it } from "vitest";
import {
  experimentBatchSpecSchema,
  experimentFidelityMetricsReportSchema,
  experimentManifestSchema,
  experimentMetricsReportSchema,
  experimentSpecSchema,
  EXPERIMENT_GOVERNANCE_BANNER,
  FIDELITY_METRICS_GOVERNANCE_BANNER,
  METRICS_GOVERNANCE_BANNER,
} from "./experimentSchema";

describe("experimentSchema", () => {
  it("parses rt_experiment_manifest_v1", () => {
    const parsed = experimentManifestSchema.parse({
      schema: "rt_experiment_manifest_v1",
      experiment_id: "exp-1",
      created_at_utc: "2026-05-26T12:00:00Z",
      governance_banner: EXPERIMENT_GOVERNANCE_BANNER,
      runs: [
        {
          run_id: "a",
          label: "run a",
          session_id: "sess-1",
          recorded_at_utc: "2026-05-26T12:01:00Z",
          snapshot: { tactical_state: { tactical_mode: "manual" } },
        },
      ],
    });
    expect(parsed.runs).toHaveLength(1);
  });

  it("parses rt_experiment_batch_v1", () => {
    const parsed = experimentBatchSpecSchema.parse({
      schema: "rt_experiment_batch_v1",
      experiment_id: "exp-batch",
      runs: [{ run_id: "r1", label: "one" }],
    });
    expect(parsed.runs[0].run_id).toBe("r1");
  });

  it("parses rt_experiment_spec_v1", () => {
    const parsed = experimentSpecSchema.parse({
      schema: "rt_experiment_spec_v1",
      experiment_id: "exp-spec",
      experiment_class: "terrain_comparison",
      governance_banner: "RT EXPERIMENT — local maintainer planning only; not operational authority",
      compile_strategy: "explicit_list",
      spec_entries: [
        {
          entry_id: "a",
          label: "a",
          template_id: "radar_north_arc_v1",
        },
      ],
    });
    expect(parsed.experiment_class).toBe("terrain_comparison");
  });

  it("parses rt_experiment_metrics_report_v1", () => {
    const parsed = experimentMetricsReportSchema.parse({
      schema: "rt_experiment_metrics_report_v1",
      experiment_id: "exp-1",
      experiment_class: "parameter_matrix",
      governance_banner: METRICS_GOVERNANCE_BANNER,
      spec_fingerprint: "abc",
      per_run_extended: [],
      compare_pairs_extended: [],
      rollup_extended: {
        class_rollup: { counts_by_class: {} },
        terrain_rollup: { ridge_counts: {}, band_counts: {} },
        visibility_rollup: { los_label_counts: {} },
        tactical_rollup: {
          mode_counts: {},
          assign_change_count: 0,
          tti_present_count: 0,
          annex_event_totals: {},
        },
        repeatability_rollup: { fingerprints: [] },
        matrix_rollup: { expected_cells: 0, populated_cells: 0, missing_cells: 0 },
      },
      handoff_eligibility: {
        experiment_level: "ineligible",
        gates: [],
        per_run_gates: [],
      },
    });
    expect(parsed.schema).toBe("rt_experiment_metrics_report_v1");
  });

  it("parses rt_experiment_fidelity_metrics_report_v1", () => {
    const parsed = experimentFidelityMetricsReportSchema.parse({
      schema: "rt_experiment_fidelity_metrics_report_v1",
      experiment_id: "exp-f5b",
      governance_banner: FIDELITY_METRICS_GOVERNANCE_BANNER,
      spec_fingerprint: "fp",
      coupling_required: true,
      per_run_fidelity: [],
      compare_pairs_fidelity: [],
      rollup_fidelity: {
        attestation_rollup: { status_counts: {} },
        divergence_rollup: { cognition_truth_divergence_count: 0 },
        repeatability_truth_rollup: { truth_fingerprints: [] },
      },
    });
    expect(parsed.schema).toBe("rt_experiment_fidelity_metrics_report_v1");
  });

  it("rejects wrong manifest schema", () => {
    expect(() =>
      experimentManifestSchema.parse({
        schema: "other",
        experiment_id: "x",
        created_at_utc: "t",
        governance_banner: "b",
        runs: [],
      }),
    ).toThrow();
  });
});
