import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";
import { parseBundleJson } from "./loadBundle";

const repoRoot = join(dirname(fileURLToPath(import.meta.url)), "../../../..");
const ridgeDemoPath = join(repoRoot, "fixtures/sa_r0/demo_ridge_defense/index.json");
const valleyDemoPath = join(repoRoot, "fixtures/sa_r0/demo_valley_ingress/index.json");

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
});
