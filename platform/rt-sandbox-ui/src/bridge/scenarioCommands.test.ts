import { describe, expect, it, vi, beforeEach } from "vitest";
import { applyScenario } from "./scenarioCommands";

describe("scenarioCommands", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("apply_scenario payload shape", async () => {
    await applyScenario("sid", {
      terrain_preset: "rt_sandbox_flat",
      assets: [{ entity_type: "radar", pose: { x: 0, y: 0, z: 10 } }],
      defenders: [],
      attackers: [{ entity_type: "drone", pose: { x: 5, y: 0, z: 10, yaw_deg: 0 } }],
    });
    const call = (fetch as ReturnType<typeof vi.fn>).mock.calls[0];
    const body = JSON.parse(String(call[1]?.body));
    expect(body.command_type).toBe("apply_scenario");
    expect(body.session_id).toBe("sid");
    expect(body.payload.terrain_preset).toBe("rt_sandbox_flat");
    expect(body.payload.assets).toHaveLength(1);
    expect(body.payload.attackers).toHaveLength(1);
    expect(body.payload.assets[0].entity_id).toBeUndefined();
  });
});
