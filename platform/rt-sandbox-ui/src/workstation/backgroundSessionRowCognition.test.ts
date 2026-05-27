import { describe, expect, it } from "vitest";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import {
  deriveBackgroundSessionRow,
  staleReasonLabel,
} from "./backgroundSessionRowCognition";

function slot(overrides: Partial<SessionSlot> = {}): SessionSlot {
  return {
    sessionId: "bg-1",
    subscriptionId: "sub",
    role: "background",
    snapshots: {
      lifecycle_state: { payload: { state: "running" }, timestamp_utc: "t0" },
      session_health: {
        payload: { telemetry_health: "stale", state: "running" },
      },
    },
    lastError: null,
    connected: true,
    lastPullUtc: "2026-05-27T10:00:00.000Z",
    drainedCount: 0,
    pulling: false,
    ...overrides,
  };
}

describe("deriveBackgroundSessionRow", () => {
  const now = Date.parse("2026-05-27T10:00:10.000Z");

  it("flags editing lock when session holds lock", () => {
    const row = deriveBackgroundSessionRow(slot(), "bg-1", now);
    expect(row.isEditingLock).toBe(true);
    expect(row.lifecycleLabel).toBe("running");
  });

  it("splits stale reasons", () => {
    const row = deriveBackgroundSessionRow(
      slot({ lastError: "PULL_FAILED" }),
      null,
      now,
    );
    expect(row.staleReasons).toContain("pull_fault");
    expect(row.staleReasons).toContain("pull_age");
    expect(row.staleReasons).toContain("telemetry");
    expect(staleReasonLabel("pull_fault")).toBe("stale: pull fault");
  });
});
