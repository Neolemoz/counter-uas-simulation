import { describe, expect, it, vi, beforeEach } from "vitest";
import { resetSession } from "./lifecycleCommands";

describe("lifecycleCommands", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("reset_session payload shape", async () => {
    await resetSession("sid");
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("reset_session");
    expect(body.session_id).toBe("sid");
    expect(body.payload).toEqual({});
  });
});
