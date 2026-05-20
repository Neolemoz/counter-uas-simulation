import { describe, expect, it } from "vitest";
import { buildVariabilitySummaries } from "./replayVariability";
import type { ReplayMcSweep } from "../sweepSchema";

const baseSweep: ReplayMcSweep = {
  artifact_type: "replay_mc_sweep_v1",
  schema_version: "replay_mc_sweep_v1",
  sweep_id: "test_sweep",
  sweep_kind: "topology_sweep",
  title: "Test",
  baseline_topology_key: "valley_ingress",
  governance: { notice: "test" },
  members: [],
  spatial_aggregate: {
    grid: { origin_enu_m: [0, 0], spacing_m: 100, size: [2, 2] },
    layers: {},
  },
  replay_narrative_summary: {
    headline: "Test headline",
    bullets: ["Narrative bullet A", "Narrative bullet B"],
  },
};

describe("buildVariabilitySummaries", () => {
  it("prefers narrative summary bullets when present", () => {
    const out = buildVariabilitySummaries(baseSweep);
    expect(out[0]).toBe("Narrative bullet A");
    expect(out).toContain("Narrative bullet B");
  });
});
