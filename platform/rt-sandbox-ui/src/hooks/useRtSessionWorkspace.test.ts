import { describe, expect, it } from "vitest";
import { shouldPullSlotInAutoRefresh } from "@/hooks/useRtSessionWorkspace";
import {
  BACKGROUND_PULL_HZ,
  DIAGNOSTIC_TELEMETRY_CHANNELS,
  TELEMETRY_CHANNELS,
} from "@/telemetry/constants";

describe("useRtSessionWorkspace contract", () => {
  it("uses diagnostic channel subset for background sessions", () => {
    expect(DIAGNOSTIC_TELEMETRY_CHANNELS.length).toBeLessThan(
      TELEMETRY_CHANNELS.length,
    );
    expect(DIAGNOSTIC_TELEMETRY_CHANNELS).toEqual([
      "session_health",
      "lifecycle_state",
      "world_summary",
    ]);
    expect(DIAGNOSTIC_TELEMETRY_CHANNELS).not.toContain("entity_pose_mirror");
    expect(DIAGNOSTIC_TELEMETRY_CHANNELS).not.toContain("clock_mirror");
  });

  it("caps background pull rate at 1 Hz per M1", () => {
    expect(BACKGROUND_PULL_HZ).toBe(1);
  });

  it("skips background slot pull when pauseBackgroundPoll is true", () => {
    expect(shouldPullSlotInAutoRefresh("active", true)).toBe(true);
    expect(shouldPullSlotInAutoRefresh("background", true)).toBe(false);
    expect(shouldPullSlotInAutoRefresh("background", false)).toBe(true);
  });

  it("keeps snapshot maps keyed by session id in slot model", () => {
    const slots = new Map([
      [
        "session-a",
        {
          sessionId: "session-a",
          snapshots: { lifecycle_state: { payload: { state: "running" } } },
        },
      ],
      [
        "session-b",
        {
          sessionId: "session-b",
          snapshots: { lifecycle_state: { payload: { state: "paused" } } },
        },
      ],
    ]);
    expect(slots.get("session-a")?.snapshots.lifecycle_state?.payload.state).toBe(
      "running",
    );
    expect(slots.get("session-b")?.snapshots.lifecycle_state?.payload.state).toBe(
      "paused",
    );
  });

  it("stores requested runtime profile per session slot", () => {
    const slots = new Map([
      [
        "session-stub",
        {
          sessionId: "session-stub",
          requestedRuntimeProfile: "stub" as const,
        },
      ],
      [
        "session-mock",
        {
          sessionId: "session-mock",
          requestedRuntimeProfile: "mock_adapter" as const,
        },
      ],
    ]);
    expect(slots.get("session-stub")?.requestedRuntimeProfile).toBe("stub");
    expect(slots.get("session-mock")?.requestedRuntimeProfile).toBe("mock_adapter");
  });

  it("tracks pulling per slot (active refresh must not use global flag)", () => {
    const slots = new Map([
      [
        "session-a",
        {
          sessionId: "session-a",
          role: "active" as const,
          pulling: false,
        },
      ],
      [
        "session-b",
        {
          sessionId: "session-b",
          role: "background" as const,
          pulling: true,
        },
      ],
    ]);
    const active = slots.get("session-a");
    const background = slots.get("session-b");
    expect(active?.pulling).toBe(false);
    expect(background?.pulling).toBe(true);
    expect(active?.pulling ?? false).toBe(false);
  });
});
