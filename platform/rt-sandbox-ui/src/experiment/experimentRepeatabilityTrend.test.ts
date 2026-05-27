import { describe, expect, it } from "vitest";
import {
  consecutiveComparePairs,
  orderRepeatabilityRuns,
  repeatFingerprintAnnotation,
} from "./experimentRepeatabilityTrend";
import type { ExperimentMetricsReport, ExperimentRun, PerRunExtended } from "./experimentSchema";

function run(id: string, repeatIndex: number | null, at: string): ExperimentRun {
  return {
    run_id: id,
    label: id,
    session_id: "sess-1",
    recorded_at_utc: at,
    snapshot: {},
    repeat_index: repeatIndex ?? undefined,
  };
}

describe("experimentRepeatabilityTrend", () => {
  it("orders by repeat_index then recorded_at_utc", () => {
    const runs = [run("c", 2, "t3"), run("a", 0, "t1"), run("b", 1, "t2")];
    const ext: PerRunExtended[] = runs.map((r) => ({
      run_id: r.run_id,
      experiment_class: "repeatability_sweep",
      spec_fingerprint: "fp",
      matrix_coords: null,
      axis_signature: null,
      repeat_group_id: "g1",
      repeat_index: r.repeat_index ?? null,
      terrain_profile_ref: null,
      nearest_ridge: null,
      elevation_band: null,
      f4_layers_enabled: [],
      los_cognition_label: null,
      occlusion_marker_count: null,
      mode_at_capture: "manual",
      assign_delta_from_prior: false,
      autonomous_pause_count: null,
      handoff_eligibility_hint: "unknown",
    }));
    const ordered = orderRepeatabilityRuns(runs, ext);
    expect(ordered.map((o) => o.run.run_id)).toEqual(["a", "b", "c"]);
  });

  it("finds consecutive compare pairs", () => {
    const pairs = consecutiveComparePairs(
      ["a", "b", "c"],
      [
        { run_id_a: "a", run_id_b: "b", badges: [{ id: "tti_delta", label: "tti_delta" }] },
        { run_id_a: "a", run_id_b: "c", badges: [] },
        { run_id_a: "b", run_id_b: "c", badges: [{ id: "visibility_label_diff", label: "visibility_label_diff" }] },
      ],
    );
    expect(pairs).toHaveLength(2);
  });

  it("annotates multi-run fingerprints", () => {
    const rollup = {
      repeatability_rollup: {
        fingerprints: [{ spec_fingerprint: "abc", run_count: 3, capture_count: 2, normalization_status_counts: {} }],
      },
    } as ExperimentMetricsReport["rollup_extended"];
    expect(repeatFingerprintAnnotation(rollup)).toContain("abc");
  });
});
