import { describe, expect, it } from "vitest";
import {
  buildBundleStorytelling,
  computeCognitionIndicators,
  importanceWeightEntries,
} from "../cognition/replayStorytelling";
import type { ReplaySaBundle } from "../bundleSchema";

const bundle = {
  comparison_hints: { topology_key: "valley_ingress" },
  scenario: { topology_tags: ["valley"], ingress_archetype: "valley" },
  clock: { duration: { start: 0, end: 50 } },
  narrative: { events: [{ category: "detection" }], windows: [{}], annotations: [{}, {}] },
  los_segments: [{ status: "terrain_blocked" }],
} as ReplaySaBundle;

describe("replayStorytelling", () => {
  it("builds bundle storytelling sections", () => {
    const sections = buildBundleStorytelling(bundle);
    expect(sections.some((s) => s.id === "what_changed")).toBe(true);
    expect(sections.some((s) => s.id === "los_instability")).toBe(true);
  });

  it("computes cognition indicators", () => {
    const indicators = computeCognitionIndicators(bundle, null);
    expect(indicators.annotationDensity).toContain("annotations");
    expect(indicators.replayPacing).toBeTruthy();
  });

  it("sorts importance weights", () => {
    const entries = importanceWeightEntries({ los: 0.9, ambiguity: 0.5 });
    expect(entries[0]?.key).toBe("los");
  });
});
