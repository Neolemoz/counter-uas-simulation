import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { MirrorEntity } from "./entityMarkers";
import {
  deriveTacticalTrajectoryGeometry,
  resolveTacticalRoleIds,
} from "./tacticalTrajectoryLayer";

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

describe("tacticalTrajectoryLayer", () => {
  it("prefers assigned ids over selected ids", () => {
    const state: TacticalStatePayload = {
      assigned_interceptor_id: "int-1",
      assigned_target_id: "tgt-1",
      selected_interceptor_id: "other",
      selected_target_id: "other-t",
    };
    expect(resolveTacticalRoleIds(state)).toEqual({
      interceptorId: "int-1",
      targetId: "tgt-1",
    });
  });

  it("builds heuristic intercept path when last_intercept_pose is present", () => {
    const state: TacticalStatePayload = {
      assigned_interceptor_id: "int-1",
      assigned_target_id: "tgt-1",
      last_intercept_pose: { x: 120, y: 30, z: 35 },
    };
    const geom = deriveTacticalTrajectoryGeometry(state, entities);
    expect(geom?.pathMode).toBe("heuristic_intercept");
    expect(geom?.pathPoints).toEqual([
      { x: 0, y: 0, z: 20 },
      { x: 120, y: 30, z: 35 },
    ]);
    expect(geom?.interceptPose).toEqual({ x: 120, y: 30, z: 35 });
  });

  it("falls back to target leg when intercept pose is absent", () => {
    const state: TacticalStatePayload = {
      selected_interceptor_id: "int-1",
      selected_target_id: "tgt-1",
    };
    const geom = deriveTacticalTrajectoryGeometry(state, entities);
    expect(geom?.pathMode).toBe("heuristic_target");
    expect(geom?.pathEndPose).toEqual({ x: 200, y: 50, z: 40 });
    expect(geom?.interceptPose).toBeNull();
  });

  it("uses predicted_path_enu_m telemetry when provided", () => {
    const state = {
      assigned_interceptor_id: "int-1",
      last_intercept_pose: { x: 120, y: 30, z: 35 },
      predicted_path_enu_m: [
        [0, 0, 20],
        [60, 15, 28],
        [120, 30, 35],
      ],
    } as TacticalStatePayload & { predicted_path_enu_m: number[][] };
    const geom = deriveTacticalTrajectoryGeometry(state, entities);
    expect(geom?.pathMode).toBe("telemetry");
    expect(geom?.pathPoints).toHaveLength(3);
  });

  it("returns null when interceptor pose is unavailable", () => {
    const state: TacticalStatePayload = {
      assigned_interceptor_id: "missing",
    };
    expect(deriveTacticalTrajectoryGeometry(state, entities)).toBeNull();
  });
});
