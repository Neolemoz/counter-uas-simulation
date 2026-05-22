import { describe, expect, it } from "vitest";
import {
  crossSweepSynthesisSchema,
  replayLinkageIndexSchema,
} from "./synthesisSchema";
import { linkageEdgesForSweep, relatedSweepIds } from "./loadSynthesis";
import type { ReplayLinkageIndex } from "./synthesisSchema";

describe("synthesisSchema", () => {
  it("parses minimal cross-sweep synthesis", () => {
    const parsed = crossSweepSynthesisSchema.parse({
      artifact_type: "cross_sweep_synthesis_v1",
      schema_version: "cross_sweep_synthesis_v1",
      sweep_ids: ["ridge_overlap_sweep"],
    });
    expect(parsed.sweep_ids).toHaveLength(1);
  });

  it("parses linkage index", () => {
    const parsed = replayLinkageIndexSchema.parse({
      artifact_type: "replay_linkage_index_v1",
      schema_version: "replay_linkage_index_v1",
      nodes: [],
      edges: [],
    });
    expect(parsed.edges).toEqual([]);
  });
});

describe("loadSynthesis helpers", () => {
  const linkage: ReplayLinkageIndex = {
    artifact_type: "replay_linkage_index_v1",
    schema_version: "replay_linkage_index_v1",
    nodes: [],
    edges: [
      {
        edge_id: "a__b__shared_pattern",
        source: "a",
        target: "b",
        link_kind: "shared_pattern",
        copy: "test",
      },
    ],
  };

  it("finds related sweeps", () => {
    expect(relatedSweepIds(linkage, "a")).toEqual(["b"]);
    expect(linkageEdgesForSweep(linkage, "a")).toHaveLength(1);
  });
});
