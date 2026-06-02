import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import {
  geometryFingerprint,
  translateLayoutToProfile,
  validateLayout,
  type RtLayoutScenarioV1,
} from "./rtLayoutMcProfile";

const GOLDEN_PATH = join(
  import.meta.dirname,
  "../../../../fixtures/rt_sandbox/rt_layout_scenario_golden_v1.json",
);

function loadGolden(): RtLayoutScenarioV1 {
  return JSON.parse(readFileSync(GOLDEN_PATH, "utf-8")) as RtLayoutScenarioV1;
}

describe("rtLayoutMcProfile", () => {
  it("validates golden fixture", () => {
    const layout = loadGolden();
    const result = validateLayout(layout as unknown as Record<string, unknown>);
    expect(result.ok).toBe(true);
    expect(result.issues).toEqual([]);
  });

  it("maps single drone and three interceptors from golden layout", () => {
    const profile = translateLayoutToProfile(loadGolden());

    expect(profile.schema_version).toBe("rt_layout_mc_profile_v1");
    expect(profile.source_layout_id).toBe("rt_layout_mc_golden");
    expect(profile.geometry_id.startsWith("rt_layout:sha256:")).toBe(true);
    expect(profile.scenario_suggestion).toBe("single");
    expect(profile.launch_args_fields).toEqual({
      target_start_x_m: "-1500",
      target_start_y_m: "0",
      target_start_z_m: "300",
      interceptor_ic_layout: "custom:-5,0,4,-4,-4,5",
    });
    expect(profile.launch_args).toContain("target_start_x_m:=-1500");
    expect(profile.launch_args).toContain(
      "interceptor_ic_layout:=custom:-5,0,4,-4,-4,5",
    );
    expect(profile.entity_counts.radar).toBe(1);
    expect(profile.entity_counts.waypoint_marker).toBe(1);
    expect(
      profile.warnings.some((w) =>
        w.includes("radar entities are retained as metadata"),
      ),
    ).toBe(true);
  });

  it("warns on multiple drones", () => {
    const layout = loadGolden();
    layout.entities.push({
      entity_type: "drone",
      pose: { x: -1200, y: 250, z: 250, yaw_deg: 0 },
    });

    const profile = translateLayoutToProfile(layout);

    expect(profile.scenario_suggestion).toBe("multi");
    expect(profile.launch_args_fields.target_start_x_m).toBeUndefined();
    expect(profile.warnings.some((w) => w.includes("multiple drone entities"))).toBe(
      true,
    );
    expect(
      profile.unsupported_fields.some((row) => row.field === "entities[drone]"),
    ).toBe(true);
  });

  it("geometry fingerprint ignores metadata notes", () => {
    const layoutA = loadGolden();
    const layoutB = loadGolden();
    layoutB.notes = "changed explanatory note";
    layoutB.created_utc = "2026-06-02T12:00:00Z";

    expect(geometryFingerprint(layoutA)).toBe(geometryFingerprint(layoutB));
  });
});
