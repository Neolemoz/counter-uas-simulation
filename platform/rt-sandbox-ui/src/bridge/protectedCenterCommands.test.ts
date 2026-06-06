import { beforeEach, describe, expect, it, vi } from "vitest";
import { designateProtectedCenter } from "./protectedCenterCommands";

describe("protectedCenterCommands", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("designate_protected_center payload shape without replace", async () => {
    await designateProtectedCenter("sid-1", "center-a");
    const call = (fetch as ReturnType<typeof vi.fn>).mock.calls[0];
    const body = JSON.parse(String(call[1]?.body));
    expect(body.command_type).toBe("designate_protected_center");
    expect(body.session_id).toBe("sid-1");
    expect(body.payload).toEqual({ entity_id: "center-a", replace: false });
  });

  it("designate_protected_center payload shape with replace", async () => {
    await designateProtectedCenter("sid-1", "center-b", true);
    const call = (fetch as ReturnType<typeof vi.fn>).mock.calls[0];
    const body = JSON.parse(String(call[1]?.body));
    expect(body.payload).toEqual({ entity_id: "center-b", replace: true });
  });
});
