import { describe, expect, it, beforeEach } from "vitest";
import { useCompareStore } from "./compareStore";
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
  clock: { domain: "log_line_index", duration: { start: 0, end: 10, step: 1 }, markers: [] },
  tracks: [],
  entities_static: [],
  zones: [],
  overlays: [],
  narrative: { events: [], annotations: [] },
  comprehension: { scan_guide: [], at_a_glance: { cards: [] } },
  source_artifacts: { embedded: true },
};

beforeEach(() => {
  useCompareStore.getState().exitCompare();
});

describe("compareStore", () => {
  it("enters compare with two bundles", () => {
    useCompareStore.getState().enterCompare(bundle, bundle);
    expect(useCompareStore.getState().mode).toBe("compare");
    expect(useCompareStore.getState().slotA.bundle?.scenario.scenario_id).toBe("x");
  });

  it("syncs clock when enabled", () => {
    useCompareStore.getState().enterCompare(bundle, bundle);
    useCompareStore.getState().setSyncClock(true);
    useCompareStore.getState().setSlotCurrentT("A", 5);
    expect(useCompareStore.getState().slotB.currentT).toBe(5);
  });
});
