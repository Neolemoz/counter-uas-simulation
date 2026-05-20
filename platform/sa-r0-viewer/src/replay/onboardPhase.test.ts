import { describe, expect, it } from "vitest";
import { getOnboardPhase } from "./onboardPhase";
import type { ReplaySaBundle } from "./bundleSchema";

function minimalBundle(overrides: Partial<ReplaySaBundle> = {}): ReplaySaBundle {
  return {
    artifact_type: "replay_sa_bundle",
    bundle_schema_version: "replay_sa_bundle_v1",
    mode: "replay_static",
    governance: { notice: "Derived evaluation artifact only." },
    lineage: {},
    scenario: { scenario_id: "t", title: "T" },
    georef_display: {
      frame: "scenario_enu",
      origin_enu_m: [0, 0, 0],
      anchor: { lat_deg: 35, lon_deg: -116, h_m: 0 },
    },
    clock: { domain: "log_line_index", duration: { start: 0, end: 10, step: 1 }, markers: [] },
    tracks: [
      {
        track_id: "interceptor_0",
        role: "interceptor",
        samples: [{ t: 3, x_m: 0, y_m: 0, z_m: 0 }],
      },
    ],
    entities_static: [],
    zones: [],
    overlays: [],
    narrative: { events: [] },
    comprehension: { at_a_glance: { cards: [], summary: {} } },
    ...overrides,
  } as ReplaySaBundle;
}

describe("getOnboardPhase", () => {
  it("returns no_samples without interceptor track", () => {
    expect(
      getOnboardPhase(minimalBundle({ tracks: [] }), 5),
    ).toBe("no_samples");
  });

  it("returns pre_launch before first sample", () => {
    expect(getOnboardPhase(minimalBundle(), 1)).toBe("pre_launch");
  });

  it("returns intercept_window on selection event", () => {
    const bundle = minimalBundle({
      narrative: {
        events: [
          { event_id: "e1", category: "selection", line_index: 4, label: "sel" },
        ],
      },
    });
    expect(getOnboardPhase(bundle, 5)).toBe("intercept_window");
  });
});
