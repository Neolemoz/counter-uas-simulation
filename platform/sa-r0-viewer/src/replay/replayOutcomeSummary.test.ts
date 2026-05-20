import { describe, expect, it } from "vitest";
import { extractReplayOutcome, formatDeltaT } from "./replayOutcomeSummary";
import type { ReplaySaBundle } from "./bundleSchema";

const bundle: ReplaySaBundle = {
  artifact_type: "replay_sa_bundle",
  bundle_schema_version: "replay_sa_bundle_v1",
  mode: "replay_static",
  governance: { notice: "test" },
  lineage: {},
  scenario: { scenario_id: "x", title: "X" },
  georef_display: {
    frame: "scenario_enu",
    origin_enu_m: [0, 0, 0],
    anchor: { lat_deg: 0, lon_deg: 0, h_m: 0 },
  },
  clock: {
    domain: "log_line_index",
    duration: { start: 2, end: 50, step: 1 },
    markers: [{ category: "detection", t: 3, event_id: "e1", label: "det" }],
  },
  tracks: [],
  entities_static: [],
  zones: [],
  overlays: [],
  narrative: {
    events: [{ event_id: "e1", category: "detection", label: "d", line_index: 3 }],
    annotations: [],
  },
  comprehension: { scan_guide: [], at_a_glance: { cards: [] } },
  source_artifacts: { embedded: true },
};

describe("extractReplayOutcome", () => {
  it("reads first detection from marker", () => {
    const s = extractReplayOutcome(bundle);
    expect(s.firstDetectionT).toBe(3);
    expect(s.durationSpan).toBe(49);
  });
});

describe("formatDeltaT", () => {
  it("formats signed delta", () => {
    expect(formatDeltaT(3, 5)).toBe("+2");
    expect(formatDeltaT(5, 3)).toBe("-2");
  });
});
