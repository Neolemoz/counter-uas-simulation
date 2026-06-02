import { describe, expect, it, vi, beforeEach } from "vitest";
import { resetSession } from "@/bridge/lifecycleCommands";
import { pauseSim, resumeSim } from "@/bridge/runtimeCommands";
import {
  resolveSelectedDefenderId,
  resolveSelectedTargetId,
} from "./useRuntimeControls";

describe("useRuntimeControls selection helpers", () => {
  const entities = [
    { entity_id: "d1", entity_type: "interceptor", pose: {} },
    { entity_id: "t1", entity_type: "drone", pose: {} },
  ];

  it("prefers selected interceptor entity for defender", () => {
    expect(resolveSelectedDefenderId("d1", entities, null)).toBe("d1");
  });

  it("falls back to tactical interceptor id", () => {
    expect(resolveSelectedDefenderId(null, entities, "d1")).toBe("d1");
  });

  it("prefers tactical target id", () => {
    expect(resolveSelectedTargetId("t1", entities, "t2")).toBe("t2");
  });

  it("uses selected drone when tactical target unset", () => {
    expect(resolveSelectedTargetId("t1", entities, null)).toBe("t1");
  });
});

describe("useRuntimeControls lifecycle commands", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("dispatches reset_session to the bridge", async () => {
    await resetSession("session-1");
    const body = JSON.parse(String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body));
    expect(body.command_type).toBe("reset_session");
    expect(body.session_id).toBe("session-1");
  });

  it("pause_sim and resume_sim command types unchanged", async () => {
    await pauseSim("session-1");
    const pauseBody = JSON.parse(
      String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body),
    );
    expect(pauseBody.command_type).toBe("pause_sim");

    await resumeSim("session-1");
    const resumeBody = JSON.parse(
      String((fetch as ReturnType<typeof vi.fn>).mock.calls[1][1]?.body),
    );
    expect(resumeBody.command_type).toBe("resume_sim");
  });
});
