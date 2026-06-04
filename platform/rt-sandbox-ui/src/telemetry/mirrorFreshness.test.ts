import { describe, expect, it } from "vitest";
import {
  deriveMirrorFreshness,
  mirrorStaleThresholdMs,
  TELEMETRY_STALE_DEFAULT_S,
} from "./mirrorFreshness";
import type { ChannelSnapshot } from "./channelIndex";

const NOW = Date.parse("2026-06-05T12:00:10.000Z");

function mirrorSnap(
  overrides: Partial<ChannelSnapshot> & { payload?: Record<string, unknown> } = {},
): ChannelSnapshot {
  return {
    channel: "entity_pose_mirror",
    timestamp_utc: "2026-06-05T12:00:09.000Z",
    payload: { entities: [{ entity_id: "e1" }], telemetry_health: "ok" },
    ...overrides,
  };
}

describe("mirrorStaleThresholdMs", () => {
  it("uses live poll cadence when hz known", () => {
    expect(mirrorStaleThresholdMs(1)).toBe(2000);
    expect(mirrorStaleThresholdMs(2)).toBe(2000);
  });

  it("falls back to bridge telemetry_stale_s default", () => {
    expect(mirrorStaleThresholdMs(null)).toBe(TELEMETRY_STALE_DEFAULT_S * 1000);
  });
});

describe("deriveMirrorFreshness", () => {
  it("returns unavailable when disconnected", () => {
    const view = deriveMirrorFreshness({
      connected: false,
      mirrorSnapshot: mirrorSnap(),
      nowMs: NOW,
    });
    expect(view.state).toBe("unavailable");
    expect(view.tone).toBe("error");
  });

  it("returns unavailable when mirror snapshot missing", () => {
    const view = deriveMirrorFreshness({ connected: true, nowMs: NOW });
    expect(view.state).toBe("unavailable");
  });

  it("returns fresh for recent mirror with ok health", () => {
    const view = deriveMirrorFreshness({
      connected: true,
      mirrorSnapshot: mirrorSnap(),
      liveBackgroundPollHz: 1,
      nowMs: NOW,
    });
    expect(view.state).toBe("fresh");
    expect(view.tone).toBe("ok");
    expect(view.label).toContain("fresh");
  });

  it("returns stale when telemetry_health is stale", () => {
    const view = deriveMirrorFreshness({
      connected: true,
      mirrorSnapshot: mirrorSnap({
        payload: { entities: [], telemetry_health: "stale" },
      }),
      nowMs: NOW,
    });
    expect(view.state).toBe("stale");
    expect(view.tone).toBe("warn");
  });

  it("returns stale when mirror event age exceeds threshold", () => {
    const view = deriveMirrorFreshness({
      connected: true,
      mirrorSnapshot: mirrorSnap({
        timestamp_utc: "2026-06-05T11:00:00.000Z",
        payload: { entities: [], telemetry_health: "ok" },
      }),
      liveBackgroundPollHz: 1,
      nowMs: NOW,
    });
    expect(view.state).toBe("stale");
  });

  it("returns unavailable on feedback_lost", () => {
    const view = deriveMirrorFreshness({
      connected: true,
      mirrorSnapshot: mirrorSnap({
        payload: { entities: [], telemetry_health: "feedback_lost" },
      }),
      nowMs: NOW,
    });
    expect(view.state).toBe("unavailable");
  });
});
