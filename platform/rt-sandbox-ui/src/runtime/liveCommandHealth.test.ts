import { describe, expect, it } from "vitest";
import { TELEMETRY_CHANNELS } from "@/telemetry/constants";
import {
  deriveLiveCommandHealth,
  isLiveCommandContext,
  LIVE_COMMAND_READY_COPY,
} from "./liveCommandHealth";

describe("isLiveCommandContext", () => {
  it("detects live from requested profile or session health", () => {
    expect(isLiveCommandContext("live", null)).toBe(true);
    expect(isLiveCommandContext(null, "live")).toBe(true);
    expect(isLiveCommandContext("stub", "stub")).toBe(false);
  });
});

describe("deriveLiveCommandHealth", () => {
  it("returns null outside live context", () => {
    expect(
      deriveLiveCommandHealth({
        connected: true,
        requestedRuntimeProfile: "stub",
        sessionState: "running",
        editingEnabled: true,
      }),
    ).toBeNull();
  });

  it("returns command_ready for connected live session", () => {
    const view = deriveLiveCommandHealth({
      connected: true,
      requestedRuntimeProfile: "live",
      sessionState: "running",
      editingEnabled: true,
      sessionHealthPayload: {
        runtime_profile: "live",
        adapter_alive: true,
        adapter_mode: "live",
      },
    });
    expect(view?.state).toBe("command_ready");
    expect(view?.tone).toBe("ok");
    expect(view?.detail).toBe(LIVE_COMMAND_READY_COPY);
  });

  it("returns command_unavailable when adapter not alive", () => {
    const view = deriveLiveCommandHealth({
      connected: true,
      requestedRuntimeProfile: "live",
      sessionState: "running",
      editingEnabled: true,
      sessionHealthPayload: {
        runtime_profile: "live",
        adapter_alive: false,
      },
    });
    expect(view?.state).toBe("command_unavailable");
    expect(view?.tone).toBe("warn");
  });

  it("returns command_unavailable when disconnected with failed preflight", () => {
    const view = deriveLiveCommandHealth({
      connected: false,
      requestedRuntimeProfile: "live",
      sessionState: "unknown",
      editingEnabled: false,
      livePreflightOk: false,
    });
    expect(view?.state).toBe("command_unavailable");
    expect(view?.detail).toContain("preflight");
  });

  it("returns command_unavailable when lifecycle blocks commands", () => {
    const view = deriveLiveCommandHealth({
      connected: true,
      requestedRuntimeProfile: "live",
      sessionState: "stopped",
      editingEnabled: true,
      sessionHealthPayload: { adapter_alive: true, runtime_profile: "live" },
    });
    expect(view?.state).toBe("command_unavailable");
  });

  it("does not add perception or parser telemetry channels", () => {
    expect(TELEMETRY_CHANNELS).not.toContain("/tracks/state");
    expect(TELEMETRY_CHANNELS).not.toContain("tracks_state");
  });
});
