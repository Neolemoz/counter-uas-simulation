import { describe, expect, it, vi, beforeEach } from "vitest";
import { deleteEntity, moveEntity, spawnAttacker, spawnEntity } from "./entityCommands";

describe("entityCommands", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("spawn_entity payload shape", async () => {
    await spawnEntity("sid", {
      entity_type: "drone",
      pose: { x: 1, y: 2, z: 10, yaw_deg: 0 },
    });
    const call = (fetch as ReturnType<typeof vi.fn>).mock.calls[0];
    const body = JSON.parse(String(call[1]?.body));
    expect(body.command_type).toBe("spawn_entity");
    expect(body.session_id).toBe("sid");
    expect(body.payload.entity_type).toBe("drone");
  });

  it("spawn_attacker payload shape", async () => {
    await spawnAttacker("sid", {
      pose: { x: 1, y: 2, z: 10, yaw_deg: 0 },
    });
    const call = (fetch as ReturnType<typeof vi.fn>).mock.calls[0];
    const body = JSON.parse(String(call[1]?.body));
    expect(body.command_type).toBe("spawn_attacker");
    expect(body.session_id).toBe("sid");
    expect(body.payload.pose.x).toBe(1);
  });

  it("move_entity payload shape", async () => {
    await moveEntity("sid", {
      entity_id: "e1",
      pose: { x: 3, y: 4, z: 10 },
    });
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("move_entity");
    expect(body.payload.entity_id).toBe("e1");
  });

  it("delete_entity payload shape", async () => {
    await deleteEntity("sid", "e1");
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("delete_entity");
    expect(body.payload.entity_id).toBe("e1");
  });
});
