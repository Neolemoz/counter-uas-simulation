import { describe, expect, it } from "vitest";
import {
  expectedPullIntervalMs,
  formatLastPullAge,
  isPullAgeStale,
} from "@/telemetry/pullAge";

describe("pullAge", () => {
  it("formats never when no last pull", () => {
    expect(formatLastPullAge(null)).toBe("never");
  });

  it("formats seconds ago", () => {
    const now = Date.parse("2026-05-27T12:00:30.000Z");
    const last = "2026-05-27T12:00:20.000Z";
    expect(formatLastPullAge(last, now)).toBe("10s ago");
  });

  it("marks background pull age stale after 2x 1 Hz interval", () => {
    const now = Date.parse("2026-05-27T12:00:05.000Z");
    const last = "2026-05-27T12:00:00.000Z";
    expect(expectedPullIntervalMs("background")).toBe(1000);
    expect(isPullAgeStale("background", last, 1, now)).toBe(true);
  });

  it("does not mark fresh background pull as stale", () => {
    const now = Date.parse("2026-05-27T12:00:01.500Z");
    const last = "2026-05-27T12:00:01.000Z";
    expect(isPullAgeStale("background", last, 1, now)).toBe(false);
  });

  it("uses active pullHz for active role stale threshold", () => {
    const now = Date.parse("2026-05-27T12:00:10.000Z");
    const fresh = "2026-05-27T12:00:09.500Z";
    const stale = "2026-05-27T12:00:00.000Z";
    expect(isPullAgeStale("active", fresh, 1, now)).toBe(false);
    expect(isPullAgeStale("active", stale, 10, now)).toBe(true);
  });
});
