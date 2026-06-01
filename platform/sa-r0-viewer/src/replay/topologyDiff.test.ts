import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";
import { computeTopologyDiff } from "./topologyDiff";
import type { ReplaySaBundle } from "./bundleSchema";
import { parseBundleJson } from "./loadBundle";

const repoRoot = join(dirname(fileURLToPath(import.meta.url)), "../../../..");
const runtimeBasePath = join(
  repoRoot,
  "fixtures/rt_visualization/runtime_capture_replay_bundle_golden_v1.json",
);
const runtimeVariantPath = join(
  repoRoot,
  "fixtures/rt_visualization/runtime_capture_replay_bundle_variant_golden_v1.json",
);

function minimalBundle(entities: ReplaySaBundle["entities_static"]): ReplaySaBundle {
  return {
    artifact_type: "replay_sa_bundle",
    bundle_schema_version: "replay_sa_bundle_v1",
    mode: "replay_static",
    governance: { notice: "test" },
    lineage: {},
    scenario: { scenario_id: "a", title: "A", topology_tags: ["valley_ingress"] },
    georef_display: {
      frame: "scenario_enu",
      origin_enu_m: [0, 0, 0],
      anchor: { lat_deg: 0, lon_deg: 0, h_m: 0 },
    },
    clock: { domain: "log_line_index", duration: { start: 0, end: 10, step: 1 }, markers: [] },
    tracks: [],
    entities_static: entities,
    zones: [],
    overlays: [],
    narrative: { events: [], annotations: [] },
    comprehension: { scan_guide: [], at_a_glance: { cards: [] } },
    source_artifacts: { embedded: true },
  };
}

describe("computeTopologyDiff", () => {
  it("detects radar position shift", () => {
    const base = minimalBundle([
      {
        entity_id: "radar_01",
        kind: "radar",
        position_enu_m: [0, 0, 14],
        label: "r",
        authoritative: false,
      },
    ]);
    const shifted = minimalBundle([
      {
        entity_id: "radar_01",
        kind: "radar",
        position_enu_m: [0, 120, 14],
        label: "r",
        authoritative: false,
      },
    ]);
    const diff = computeTopologyDiff(base, shifted);
    expect(diff.bullets.some((b) => b.includes("radar_01"))).toBe(true);
    expect(diff.highlight.entityIds).toContain("radar_01");
  });

  it("detects runtime replay entity shift", () => {
    const base = parseBundleJson(readFileSync(runtimeBasePath, "utf-8"));
    const variant = parseBundleJson(readFileSync(runtimeVariantPath, "utf-8"));
    const diff = computeTopologyDiff(base, variant);
    expect(diff.highlight.entityIds).toContain("defender-alpha");
    expect(diff.bullets.some((b) => b.includes("defender-alpha"))).toBe(true);
  });
});
