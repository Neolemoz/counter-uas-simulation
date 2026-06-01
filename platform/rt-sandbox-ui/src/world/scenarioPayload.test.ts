import { describe, expect, it } from "vitest";
import {
  entitiesToScenarioPayload,
  validateScenarioCaps,
} from "./scenarioPayload";

describe("scenarioPayload", () => {
  it("groups entities into bridge scenario arrays", () => {
    const payload = entitiesToScenarioPayload([
      {
        entity_id: "a1",
        entity_type: "radar",
        pose: { x: 1, y: 2, z: 10, yaw_deg: 45 },
      },
      {
        entity_id: "d1",
        entity_type: "interceptor",
        pose: { x: 3, y: 4, z: 10 },
      },
      {
        entity_id: "t1",
        entity_type: "drone",
        pose: { x: 500, y: -500, z: 200 },
      },
    ]);

    expect(payload.terrain_preset).toBe("rt_sandbox_flat");
    expect(payload.assets).toHaveLength(1);
    expect(payload.defenders).toHaveLength(1);
    expect(payload.attackers).toHaveLength(1);
    expect(payload.assets[0]).toEqual({
      entity_type: "radar",
      pose: { x: 1, y: 2, z: 10, yaw_deg: 45 },
    });
    expect(payload.attackers[0].pose).toEqual({ x: 500, y: -500, z: 200 });
    expect(
      Object.prototype.hasOwnProperty.call(payload.assets[0], "entity_id"),
    ).toBe(false);
  });

  it("validateScenarioCaps rejects empty layout", () => {
    expect(validateScenarioCaps([]).ok).toBe(false);
  });

  it("validateScenarioCaps rejects unknown types", () => {
    const result = validateScenarioCaps([
      { entity_id: "x", entity_type: "unknown", pose: { x: 0, y: 0, z: 0 } },
    ]);
    expect(result.ok).toBe(false);
  });
});
