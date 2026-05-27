import { describe, expect, it } from "vitest";
import {
  applyLocalEntityCommand,
  countEntitiesByType,
  mergeTelemetryAndLocalEntities,
  pruneLocalEntities,
} from "./localEntityMirror";

describe("localEntityMirror", () => {
  it("adds a successful spawn from command response before telemetry arrives", () => {
    const local = applyLocalEntityCommand(
      {},
      "spawn_entity",
      { ok: true, entity_id: "e1" },
      {
        entityType: "drone",
        pose: { x: 25, y: -25, z: 10, yaw_deg: 0 },
      },
    );

    expect(local.e1.entity_type).toBe("drone");
    expect(local.e1.pose.x).toBe(25);
  });

  it("keeps local command entity visible until telemetry contains it", () => {
    const local = {
      e1: {
        entity_id: "e1",
        entity_type: "drone",
        pose: { x: 1, y: 2, z: 10 },
      },
    };

    expect(mergeTelemetryAndLocalEntities([], local, new Set())).toHaveLength(1);
    expect(pruneLocalEntities(local, [{ ...local.e1, pose: { x: 3 } }])).toEqual(
      {},
    );
  });

  it("filters stale telemetry for locally deleted entities", () => {
    const merged = mergeTelemetryAndLocalEntities(
      [
        {
          entity_id: "e1",
          entity_type: "drone",
          pose: { x: 1, y: 2, z: 10 },
        },
      ],
      {},
      new Set(["e1"]),
    );

    expect(merged).toEqual([]);
  });

  it("counts a spawn immediately from merged local entities", () => {
    const local = applyLocalEntityCommand(
      {},
      "spawn_entity",
      { ok: true, entity_id: "radar-1" },
      {
        entityType: "radar",
        pose: { x: 1, y: 2, z: 10 },
      },
    );
    const merged = mergeTelemetryAndLocalEntities([], local, new Set());

    expect(countEntitiesByType(merged).radar).toBe(1);
  });

  it("decrements count immediately after local delete despite stale telemetry", () => {
    const telemetryEntity = {
      entity_id: "radar-1",
      entity_type: "radar",
      pose: { x: 1, y: 2, z: 10 },
    };
    const merged = mergeTelemetryAndLocalEntities(
      [telemetryEntity],
      applyLocalEntityCommand(
        { "radar-1": telemetryEntity },
        "delete_entity",
        { ok: true },
        { entityId: "radar-1" },
      ),
      new Set(["radar-1"]),
    );

    expect(countEntitiesByType(merged).radar ?? 0).toBe(0);
  });

  it("keeps count stable when telemetry reconciles a local spawn", () => {
    const local = applyLocalEntityCommand(
      {},
      "spawn_entity",
      { ok: true, entity_id: "radar-1" },
      { entityType: "radar", pose: { x: 1, y: 2, z: 10 } },
    );
    const telemetryEntity = {
      entity_id: "radar-1",
      entity_type: "radar",
      pose: { x: 1, y: 2, z: 10 },
    };
    const reconciledLocal = pruneLocalEntities(local, [telemetryEntity]);
    const merged = mergeTelemetryAndLocalEntities(
      [telemetryEntity],
      reconciledLocal,
      new Set(),
    );

    expect(countEntitiesByType(merged).radar).toBe(1);
  });
});
