import { describe, expect, it } from "vitest";
import { replayMcSweepSchema, sweepsIndexSchema } from "./sweepSchema";

describe("sweepSchema", () => {
  it("parses minimal sweep shape", () => {
    const sample = {
      artifact_type: "replay_mc_sweep_v1",
      schema_version: "replay_mc_sweep_v1",
      sweep_id: "test_sweep",
      sweep_kind: "topology_sweep",
      title: "Test",
      baseline_topology_key: "ridge_defense",
      governance: { notice: "explanatory" },
      members: [
        {
          member_id: "m0",
          pack_id: "ridge_defense",
          demo_bundle_url: "/demo/index.json",
        },
      ],
      spatial_aggregate: {
        grid: { origin_enu_m: [0, 0], spacing_m: 100, size: [2, 2] },
        layers: {
          ambiguity_density: { counts: [0, 0, 0, 0] },
        },
      },
    };
    const parsed = replayMcSweepSchema.parse(sample);
    expect(parsed.sweep_id).toBe("test_sweep");
  });

  it("parses D3 narrative and cohort fields", () => {
    const sample = {
      artifact_type: "replay_mc_sweep_v1",
      schema_version: "replay_mc_sweep_v1",
      sweep_id: "test_sweep",
      sweep_kind: "topology_sweep",
      title: "Test",
      baseline_topology_key: "ridge_defense",
      governance: { notice: "explanatory" },
      members: [
        {
          member_id: "m0",
          pack_id: "ridge_defense",
          demo_bundle_url: "/demo/index.json",
          replay_pattern_tags: ["los_fragmented_replay"],
        },
      ],
      spatial_aggregate: {
        grid: { origin_enu_m: [0, 0], spacing_m: 100, size: [2, 2] },
        layers: { ambiguity_density: { counts: [1] } },
      },
      replay_narrative_summary: {
        headline: "h",
        bullets: ["b"],
      },
      replay_cohorts: [
        {
          cohort_id: "c0",
          label: "Cohort",
          pattern_tags: ["los_fragmented_replay"],
          member_indices: [0],
          dominant_summary: "s",
        },
      ],
    };
    const parsed = replayMcSweepSchema.parse(sample);
    expect(parsed.replay_narrative_summary?.headline).toBe("h");
    expect(parsed.replay_cohorts?.[0]?.cohort_id).toBe("c0");
  });

  it("parses sweeps index", () => {
    const idx = sweepsIndexSchema.parse({
      artifact_type: "scenario_sweeps_index_v1",
      schema_version: "scenario_sweeps_index_v1",
      sweeps: [],
    });
    expect(idx.sweeps).toEqual([]);
  });
});
