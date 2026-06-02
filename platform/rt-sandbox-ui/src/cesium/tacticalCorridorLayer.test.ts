import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { MirrorEntity } from "./entityMarkers";
import {
  clampCorridorPoints,
  resolveThreatCorridorForRender,
} from "./tacticalCorridorLayer";

const entities: MirrorEntity[] = [
  {
    entity_id: "int-1",
    entity_type: "interceptor",
    pose: { x: 0, y: 0, z: 20 },
  },
  {
    entity_id: "tgt-1",
    entity_type: "drone",
    pose: { x: 200, y: 50, z: 40 },
  },
];

describe("tacticalCorridorLayer", () => {
  it("returns null when target and solution geometry are unavailable", () => {
    expect(
      resolveThreatCorridorForRender(
        { assigned_interceptor_id: "int-1" },
        entities,
      ),
    ).toBeNull();
  });

  it("resolves heuristic corridor from assigned target to solution point", () => {
    const state: TacticalStatePayload = {
      assigned_interceptor_id: "int-1",
      assigned_target_id: "tgt-1",
      last_intercept_pose: { x: 120, y: 30, z: 35 },
    };
    const threat = resolveThreatCorridorForRender(state, entities);
    expect(threat?.mode).toBe("direct_fallback");
    expect(threat?.corridorPoints[0]).toEqual({ x: 200, y: 50, z: 40 });
    expect(threat?.corridorPoints.at(-1)).toEqual({ x: 120, y: 30, z: 35 });
  });

  it("clamps long telemetry paths to the decor entity budget", () => {
    const dense = Array.from({ length: 120 }, (_, i) => ({
      x: i,
      y: 0,
      z: 10,
    }));
    const clamped = clampCorridorPoints(dense, 48);
    expect(clamped.length).toBeLessThanOrEqual(48);
    expect(clamped[0]).toEqual(dense[0]);
    expect(clamped.at(-1)).toEqual(dense.at(-1));
  });
});
