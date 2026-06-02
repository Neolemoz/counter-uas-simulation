import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import {
  resolveComparisonSessionId,
  resolveSessionCompareContext,
  resolveTacticalCompareContext,
} from "./tacticalCompareContext";

function slot(
  sessionId: string,
  tacticalState: Record<string, unknown> | null,
): SessionSlot {
  return {
    sessionId,
    subscriptionId: `sub-${sessionId}`,
    role: sessionId.startsWith("active") ? "active" : "background",
    connected: true,
    lastError: null,
    lastPullUtc: null,
    drainedCount: 0,
    pulling: false,
    snapshots: {
      tactical_state: tacticalState
        ? {
            channel: "tactical_state",
            timestamp_utc: "2026-01-01T00:00:00Z",
            payload: {
              schema: "rt_tactical_state_v1",
              ...tacticalState,
            },
          }
        : undefined,
      entity_pose_mirror: {
        channel: "entity_pose_mirror",
        timestamp_utc: "2026-01-01T00:00:00Z",
        payload: {
          entities: [
            {
              entity_id: "tgt-1",
              entity_type: "drone",
              pose: { x: 10, y: 0, z: 20 },
            },
            {
              entity_id: "int-1",
              entity_type: "interceptor",
              pose: { x: 0, y: 0, z: 20 },
            },
          ],
        },
      },
    },
  };
}

describe("tacticalCompareContext", () => {
  it("picks the non-active session for multi-session compare", () => {
    expect(
      resolveComparisonSessionId("active-a", ["active-a", "background-b"]),
    ).toBe("background-b");
  });

  it("resolves background session tactical state and entities", () => {
    const context = resolveSessionCompareContext(
      "active-a",
      ["active-a", "background-b"],
      [
        slot("active-a", null),
        slot("background-b", {
          assigned_interceptor_id: "int-1",
          assigned_target_id: "tgt-1",
          last_intercept_pose: { x: 50, y: 0, z: 25 },
        }),
      ],
    );
    expect(context?.source).toBe("session");
    expect(context?.compareSessionId).toBe("background-b");
    expect(context?.compareState.assigned_target_id).toBe("tgt-1");
    expect(context?.compareEntities).toHaveLength(2);
  });

  it("prefers embedded compare state when present", () => {
    const embedded: TacticalStatePayload = {
      assigned_target_id: "tgt-embedded",
    };
    const context = resolveTacticalCompareContext({
      currentState: {
        compare_tactical_state: embedded,
      } as TacticalStatePayload & { compare_tactical_state: TacticalStatePayload },
      activeSessionId: "active-a",
      orderedSessionIds: ["active-a", "background-b"],
      slots: [],
      activeEntities: [],
    });
    expect(context?.source).toBe("embedded");
    expect(context?.compareState.assigned_target_id).toBe("tgt-embedded");
  });
});
