import { describe, expect, it, vi, beforeEach } from "vitest";
import { moveEntity } from "./entityCommands";

/** Phase C3: move_entity contract unchanged — grid/globe drag uses this path. */
describe("move_entity compatibility", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("move_entity payload unchanged", async () => {
    await moveEntity("sid", {
      entity_id: "e1",
      pose: { x: 10, y: 20, z: 10 },
    });
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("move_entity");
    expect(body.payload.entity_id).toBe("e1");
  });
});
