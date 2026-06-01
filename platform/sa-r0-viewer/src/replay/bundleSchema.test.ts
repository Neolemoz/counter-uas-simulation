import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";
import { parseBundleJson } from "./loadBundle";

const repoRoot = join(dirname(fileURLToPath(import.meta.url)), "../../../..");
const ridgeDemoPath = join(repoRoot, "fixtures/sa_r0/demo_ridge_defense/index.json");
const valleyDemoPath = join(repoRoot, "fixtures/sa_r0/demo_valley_ingress/index.json");
const runtimeReplayGoldenPath = join(
  repoRoot,
  "fixtures/rt_visualization/runtime_capture_replay_bundle_golden_v1.json",
);
const rtTacticalDemoPath = join(
  repoRoot,
  "platform/sa-r0-viewer/public/demo/rt_tactical_continuity/index.json",
);

describe("replay_sa_bundle_v1", () => {
  it("parses committed ridge demo bundle without Zod errors", () => {
    const text = readFileSync(ridgeDemoPath, "utf-8");
    const bundle = parseBundleJson(text);
    expect(bundle.comprehension.at_a_glance.cards.length).toBeGreaterThan(0);
    expect(bundle.comprehension.at_a_glance.summary).toBeDefined();
    expect(bundle.los_segments?.length).toBeGreaterThan(0);
    expect(bundle.scenario.terrain_model?.type).toBe("fictional_heightmap");
    expect(bundle.scenario.scenario_pack_id).toBe("ridge_defense_demo");
    const src = bundle as { source_artifacts?: { scenario_pack?: string } };
    expect(src.source_artifacts?.scenario_pack).toBe("fixtures/scenarios/ridge_defense");
  });

  it("parses valley ingress demo with LOS segments", () => {
    const text = readFileSync(valleyDemoPath, "utf-8");
    const bundle = parseBundleJson(text);
    expect(bundle.scenario.topology_tags).toContain("valley_ingress");
    expect(bundle.los_segments?.length).toBeGreaterThan(0);
  });

  it("parses RT runtime capture replay bundle golden", () => {
    const bundle = parseBundleJson(readFileSync(runtimeReplayGoldenPath, "utf-8"));
    expect(bundle.bundle_schema_version).toBe("replay_sa_bundle_v1");
    expect(bundle.clock.domain).toBe("runtime_capture_frame_index");
    expect(bundle.tracks.length).toBeGreaterThan(0);
    expect(bundle.panels?.telemetry_series?.length).toBeGreaterThan(0);
  });

  it("parses RT tactical continuity demo with embedded annex", () => {
    const bundle = parseBundleJson(readFileSync(rtTacticalDemoPath, "utf-8"));
    const block = bundle.rt_tactical_replay_continuity;
    expect(block?.continuity_available).toBe(true);
    expect(block?.schema).toBe("rt_tactical_replay_continuity_v1");
    expect(block?.tactical_annex?.authority_label).toBe("replay_boundary_scoped");
    expect(bundle.rt_tactical_replay_continuity).toBeDefined();
  });

  it("parses ridge demo without rt_tactical_replay_continuity (backward compat)", () => {
    const bundle = parseBundleJson(readFileSync(ridgeDemoPath, "utf-8"));
    expect(bundle.rt_tactical_replay_continuity).toBeUndefined();
  });
});
