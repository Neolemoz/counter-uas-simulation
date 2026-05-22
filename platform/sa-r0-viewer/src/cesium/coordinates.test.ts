import { describe, expect, it } from "vitest";
import { replaySaBundleSchema } from "../replay/bundleSchema";
import { enuToCartographic } from "./coordinates";

const minimalBundle = replaySaBundleSchema.parse({
  artifact_type: "replay_sa_bundle",
  bundle_schema_version: "replay_sa_bundle_v1",
  mode: "replay_static",
  governance: { notice: "Derived evaluation artifact only. Test." },
  lineage: {},
  scenario: { scenario_id: "t", title: "T" },
  georef_display: {
    frame: "scenario_enu",
    origin_enu_m: [0, 0, 0],
    anchor: { lat_deg: 35, lon_deg: -116, h_m: 0 },
    caveat: "test",
  },
  clock: { domain: "log_line_index", duration: { start: 0, end: 10, step: 1 }, markers: [] },
  tracks: [],
  entities_static: [],
  zones: [],
  overlays: [],
  narrative: { events: [] },
  comprehension: {
    at_a_glance: { cards: [], summary: {} },
  },
});

describe("enuToCartographic", () => {
  it("maps origin near anchor", () => {
    const c = enuToCartographic(minimalBundle, 0, 0, 0);
    expect(c.height).toBe(0);
    expect(c.latitude).toBeCloseTo((35 * Math.PI) / 180, 4);
    expect(c.longitude).toBeCloseTo((-116 * Math.PI) / 180, 4);
  });
});
