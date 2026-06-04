import { beforeEach, describe, expect, it, vi } from "vitest";
import { stopSessionForProfile, stopSim } from "./sessionStop";

describe("sessionStop", () => {
  beforeEach(() => {
    vi.stubGlobal("fetch", vi.fn(async () => ({
      ok: true,
      json: async () => ({ ok: true, error_code: "OK" }),
    })));
    vi.stubGlobal("crypto", { randomUUID: () => "test-uuid" });
  });

  it("stopSim dispatches stop_sim terminate path", async () => {
    await stopSim("session-live");
    const body = JSON.parse(
      String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body),
    );
    expect(body.command_type).toBe("stop_sim");
    expect(body.session_id).toBe("session-live");
  });

  it("live profile uses stop_sim", async () => {
    await stopSessionForProfile("session-live", "live");
    const body = JSON.parse(
      String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body),
    );
    expect(body.command_type).toBe("stop_sim");
  });

  it("stub profile uses stop_session pause path", async () => {
    await stopSessionForProfile("session-stub", "stub");
    const body = JSON.parse(
      String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body),
    );
    expect(body.command_type).toBe("stop_session");
  });

  it("mock_adapter profile uses stop_session pause path", async () => {
    await stopSessionForProfile("session-mock", "mock_adapter");
    const body = JSON.parse(
      String((fetch as ReturnType<typeof vi.fn>).mock.calls[0][1]?.body),
    );
    expect(body.command_type).toBe("stop_session");
  });
});
