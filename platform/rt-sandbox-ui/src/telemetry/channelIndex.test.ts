import { describe, expect, it } from "vitest";
import {
  indexLatestByChannel,
  mergeChannelSnapshots,
  sessionStateFromSnapshots,
} from "@/telemetry/channelIndex";

describe("channelIndex", () => {
  it("keeps latest event per channel in batch order", () => {
    const events = [
      {
        channel: "world_summary",
        session_id: "s1",
        timestamp_utc: "t1",
        payload: { revision: 1 },
      },
      {
        channel: "world_summary",
        session_id: "s1",
        timestamp_utc: "t2",
        payload: { revision: 2 },
      },
      {
        channel: "lifecycle_state",
        session_id: "s1",
        timestamp_utc: "t3",
        payload: { state: "running" },
      },
    ];
    const indexed = indexLatestByChannel(events);
    expect(indexed.world_summary?.payload.revision).toBe(2);
    expect(indexed.lifecycle_state?.payload.state).toBe("running");
  });

  it("merges snapshots with incoming winning", () => {
    const existing = {
      world_summary: {
        channel: "world_summary" as const,
        timestamp_utc: "t1",
        payload: { revision: 1 },
      },
    };
    const merged = mergeChannelSnapshots(existing, [
      {
        channel: "world_summary",
        session_id: "s1",
        timestamp_utc: "t2",
        payload: { revision: 3 },
      },
    ]);
    expect(merged.world_summary?.payload.revision).toBe(3);
  });

  it("derives session state from lifecycle then health", () => {
    expect(
      sessionStateFromSnapshots({
        lifecycle_state: {
          channel: "lifecycle_state",
          timestamp_utc: "t",
          payload: { state: "paused" },
        },
      }),
    ).toBe("paused");
    expect(
      sessionStateFromSnapshots({
        session_health: {
          channel: "session_health",
          timestamp_utc: "t",
          payload: { state: "running" },
        },
      }),
    ).toBe("running");
  });
});
