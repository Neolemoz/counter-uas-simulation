import { describe, expect, it } from "vitest";
import {
  entitiesFromSnapshot,
  mergeChannelSnapshots,
  type TelemetryEvent,
} from "./channelIndex";

/** Validates entity_pose_mirror updates propagate to entity lists (Cesium input path). */
describe("entity_pose_mirror propagation", () => {
  const sid = "session-test";

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

  it("merges updated mirror poses into snapshot entities", () => {
    const initial = mergeChannelSnapshots(
      {},
      [mirrorEvent([{ entity_id: "d1", entity_type: "drone", pose: { x: 1, y: 2, z: 3 } }], "2026-06-05T12:00:01.000Z")],
    );
    const updated = mergeChannelSnapshots(initial, [
      mirrorEvent(
        [{ entity_id: "d1", entity_type: "drone", pose: { x: 10, y: 20, z: 30 } }],
        "2026-06-05T12:00:02.000Z",
      ),
    ]);

    const before = entitiesFromSnapshot(initial.entity_pose_mirror);
    const after = entitiesFromSnapshot(updated.entity_pose_mirror);

    expect(before[0]?.pose).toEqual({ x: 1, y: 2, z: 3 });
    expect(after[0]?.pose).toEqual({ x: 10, y: 20, z: 30 });
    expect(updated.entity_pose_mirror?.timestamp_utc).toBe(
      "2026-06-05T12:00:02.000Z",
    );
  });

  it("adds entities when mirror gains new entries", () => {
    const snap = mergeChannelSnapshots({}, [
      mirrorEvent(
        [
          { entity_id: "d1", entity_type: "drone", pose: { x: 0, y: 0, z: 5 } },
          { entity_id: "i1", entity_type: "interceptor", pose: { x: 5, y: 5, z: 5 } },
        ],
        "2026-06-05T12:00:03.000Z",
      ),
    ]);
    expect(entitiesFromSnapshot(snap.entity_pose_mirror)).toHaveLength(2);
  });
});
