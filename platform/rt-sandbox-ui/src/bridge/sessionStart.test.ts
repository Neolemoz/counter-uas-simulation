import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { startSession } from "./client";

describe("startSession runtime profile wiring", () => {
  beforeEach(() => {
    vi.stubGlobal(
      "fetch",
      vi.fn().mockResolvedValue({
        ok: true,
        json: async () => ({ ok: true, session_id: "sess-1" }),
      }),
    );
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("sends no payload for stub runtime (default)", async () => {
    await startSession();
    const fetchMock = vi.mocked(fetch);
    const body = JSON.parse(String(fetchMock.mock.calls[0]?.[1]?.body));
    expect(body.command_type).toBe("start_session");
    expect(body.payload).toBeUndefined();
  });

  it("sends mock_adapter runtime_profile payload", async () => {
    await startSession({ runtimeProfile: "mock_adapter" });
    const fetchMock = vi.mocked(fetch);
    const body = JSON.parse(String(fetchMock.mock.calls[0]?.[1]?.body));
    expect(body.payload).toEqual({ runtime_profile: "mock_adapter" });
  });
});
