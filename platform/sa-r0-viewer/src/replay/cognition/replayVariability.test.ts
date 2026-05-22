import { describe, expect, it } from "vitest";
import { buildVariabilitySummaries } from "./replayVariability";
import type { ReplayMcSweep } from "../sweepSchema";

const baseSweep = {
  artifact_type: "replay_mc_sweep_v1" as const,
  schema_version: "replay_mc_sweep_v1" as const,
  sweep_id: "t",
  sweep_kind: "topology_sweep" as const,
  title: "T",
  baseline_topology_key: "ridge_defense",
  governance: { notice: "n" },
  members: [{ member_id: "m", pack_id: "ridge_defense", demo_bundle_url: "/x" }],
  spatial_aggregate: {
    grid: { origin_enu_m: [0, 0], spacing_m: 100, size: [2, 2] },
    layers: {},
  },
  replay_aggregation: {
    outcome_histogram: { first_detection_t: [3, 8, 4] },
    dominant_patterns: ["Pattern A."],
  },
};

describe("replayVariability", () => {
  it("includes dominant patterns and detection spread", () => {
    const lines = buildVariabilitySummaries(baseSweep as ReplayMcSweep);
    expect(lines.some((l) => l.includes("Pattern A"))).toBe(true);
    expect(lines.some((l) => l.includes("Detection timing"))).toBe(true);
  });
});
