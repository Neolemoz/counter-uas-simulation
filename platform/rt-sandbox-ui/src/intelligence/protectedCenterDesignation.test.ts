import { beforeEach, describe, expect, it, vi } from "vitest";
import { designateProtectedCenter } from "@/bridge/protectedCenterCommands";
import {
  executeProtectedCenterDesignation,
  replaceProtectedCenterConfirmMessage,
  resolveProtectedCenterDesignationAttempt,
} from "./protectedCenterDesignation";

describe("protected center designation flow", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("successful designate sends replace when switching centers", async () => {
    const result = await designateProtectedCenter("sid", "center-b", true);
    expect(result.ok).toBe(true);
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.payload.replace).toBe(true);
    expect(body.payload.entity_id).toBe("center-b");
  });

  it("resolve attempt requires explicit selection and blocks when already designated", () => {
    expect(
      resolveProtectedCenterDesignationAttempt({
        sessionId: "sid",
        entityId: "center-a",
        currentCenterId: "center-a",
        editingEnabled: true,
        busy: false,
      }),
    ).toBeNull();
  });

  it("replace confirmation path uses confirm before replace", () => {
    const confirm = vi.fn(() => false);
    const attempt = resolveProtectedCenterDesignationAttempt({
      sessionId: "sid",
      entityId: "center-b",
      currentCenterId: "center-a",
      editingEnabled: true,
      busy: false,
      confirm,
    });
    expect(confirm).toHaveBeenCalledWith(
      replaceProtectedCenterConfirmMessage("center-a", "center-b"),
    );
    expect(attempt).toBeNull();
  });

  it("executeProtectedCenterDesignation uses same command payload as grid flow", async () => {
    const designate = vi.fn(async () => ({ ok: true, error_code: "OK" }));
    const result = await executeProtectedCenterDesignation(
      { sessionId: "sid", entityId: "center-b", replace: true },
      designate,
    );
    expect(result.ok).toBe(true);
    expect(designate).toHaveBeenCalledWith("sid", "center-b", true);
  });

  it("does not infer designation from selection without explicit center id", () => {
    const attempt = resolveProtectedCenterDesignationAttempt({
      sessionId: "sid",
      entityId: "wp-a",
      currentCenterId: null,
      editingEnabled: true,
      busy: false,
    });
    expect(attempt).toEqual({
      sessionId: "sid",
      entityId: "wp-a",
      replace: false,
    });
  });
});
