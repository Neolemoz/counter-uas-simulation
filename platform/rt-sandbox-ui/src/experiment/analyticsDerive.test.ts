import { describe, expect, it } from "vitest";
import {
  buildComparePairs,
  deriveExperimentAnalytics,
  rollupFromPerRun,
} from "./analyticsDerive";
import type { ExperimentManifest } from "./experimentSchema";

const baseManifest: ExperimentManifest = {
  schema: "rt_experiment_manifest_v1",
  experiment_id: "exp-test",
  created_at_utc: "2026-05-26T12:00:00Z",
  governance_banner: "RT EXPERIMENT — explanatory compare only; not operational authority",
  runs: [
    {
      run_id: "run-b",
      label: "B",
      session_id: "sess-bbbbbbb",
      recorded_at_utc: "2026-05-26T12:01:00Z",
      snapshot: {
        tactical_state: { tactical_mode: "assisted", tti_s: 20 },
        world_summary: { entity_count: 2 },
      },
    },
    {
      run_id: "run-a",
      label: "A",
      session_id: "sess-aaaaaaa",
      recorded_at_utc: "2026-05-26T12:00:00Z",
      capture_candidate_id: "cap-1",
      snapshot: {
        tactical_state: { tactical_mode: "manual", tti_s: 10 },
        world_summary: { entity_count: 3 },
      },
    },
  ],
};

describe("analyticsDerive", () => {
  it("sorts per_run by run_id deterministically", () => {
    const report = deriveExperimentAnalytics(baseManifest);
    expect(report.per_run.map((r) => r.run_id)).toEqual(["run-a", "run-b"]);
  });

  it("computes rollup counts without forbidden fields", () => {
    const report = deriveExperimentAnalytics(baseManifest);
    expect(report.rollup.run_count).toBe(2);
    expect(report.rollup.capture_count).toBe(1);
    expect(report.rollup.mode_counts.manual).toBe(1);
    expect(report.rollup.mode_counts.assisted).toBe(1);
    const keys = Object.keys(report.rollup as object);
    expect(keys).not.toContain("success_rate");
    expect(keys).not.toContain("readiness_index");
  });

  it("emits compare pair badges for mode change", () => {
    const pairs = buildComparePairs(baseManifest);
    expect(pairs).toHaveLength(1);
    expect(pairs[0].badges.map((b) => b.id)).toContain("mode_changed");
  });

  it("produces identical reports on repeat derive", () => {
    const a = deriveExperimentAnalytics(baseManifest);
    const b = deriveExperimentAnalytics(baseManifest);
    expect(a.per_run).toEqual(b.per_run);
    expect(a.compare_pairs).toEqual(b.compare_pairs);
    expect(a.rollup).toEqual(b.rollup);
  });

  it("joins batch spec dwell and template", () => {
    const report = deriveExperimentAnalytics(baseManifest, {
      schema: "rt_experiment_batch_v1",
      experiment_id: "exp-test",
      runs: [{ run_id: "run-a", label: "A", template_id: "radar_north_arc_v1", dwell_s: 5 }],
    });
    const rowA = report.per_run.find((r) => r.run_id === "run-a");
    expect(rowA?.template_id).toBe("radar_north_arc_v1");
    expect(rowA?.dwell_s).toBe(5);
  });

  it("rollupFromPerRun handles empty", () => {
    expect(rollupFromPerRun([])).toEqual({
      run_count: 0,
      capture_count: 0,
      mode_counts: {},
      template_ids_used: [],
    });
  });
});
