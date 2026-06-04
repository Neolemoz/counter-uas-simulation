import { describe, expect, it } from "vitest";
import {
  entitiesFromSnapshot,
  mergeChannelSnapshots,
  type TelemetryEvent,
} from "./channelIndex";

/** Command result visibility via entity_pose_mirror (Step 5). */
describe("command path mirror visibility", () => {
  const sid = "session-cmd";

  function mirrorEvent(
    entities: Array<Record<string, unknown>>,
    timestamp_utc: string,
  ): TelemetryEvent {
    return {
      channel: "entity_pose_mirror",
      session_id: sid,
      timestamp_utc,
      payload: {
        entities,
        telemetry_health: "ok",
        source: "adapter_feedback",
      },
    };
  }

  it("spawn visible after command publish + pull merge", () => {
    const afterSpawn = mergeChannelSnapshots({}, [
      mirrorEvent(
        [{ entity_id: "d1", entity_type: "drone", pose: { x: 1, y: 2, z: 10 } }],
        "2026-06-05T12:00:01.000Z",
      ),
    ]);
    const entities = entitiesFromSnapshot(afterSpawn.entity_pose_mirror);
    expect(entities).toHaveLength(1);
    expect(entities[0]?.entity_id).toBe("d1");
  });

  it("move visible when mirror pose updates on pull", () => {
    const initial = mergeChannelSnapshots({}, [
      mirrorEvent(
        [{ entity_id: "d1", entity_type: "drone", pose: { x: 1, y: 2, z: 10 } }],
        "2026-06-05T12:00:01.000Z",
      ),
    ]);
    const afterMove = mergeChannelSnapshots(initial, [
      mirrorEvent(
        [{ entity_id: "d1", entity_type: "drone", pose: { x: 50, y: 60, z: 15 } }],
        "2026-06-05T12:00:02.000Z",
      ),
    ]);
    const pose = entitiesFromSnapshot(afterMove.entity_pose_mirror)[0]?.pose as
      | Record<string, unknown>
      | undefined;
    expect(pose?.x).toBe(50);
    expect(pose?.y).toBe(60);
  });

  it("delete visible when mirror entity list empties on pull", () => {
    const initial = mergeChannelSnapshots({}, [
      mirrorEvent(
        [{ entity_id: "d1", entity_type: "drone", pose: { x: 0, y: 0, z: 5 } }],
        "2026-06-05T12:00:01.000Z",
      ),
    ]);
    const afterDelete = mergeChannelSnapshots(initial, [
      mirrorEvent([], "2026-06-05T12:00:03.000Z"),
    ]);
    expect(entitiesFromSnapshot(afterDelete.entity_pose_mirror)).toHaveLength(0);
  });
});
