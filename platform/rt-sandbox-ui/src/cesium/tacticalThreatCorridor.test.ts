import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { MirrorEntity } from "./entityMarkers";
import {
  buildCorridorRibbonPolygon,
  deriveThreatCorridorGeometry,
} from "./tacticalThreatCorridor";
import { deriveTacticalTrajectoryGeometry } from "./tacticalTrajectoryLayer";

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

describe("tacticalThreatCorridor", () => {
  it("uses threat_path_enu_m when provided", () => {
    const state = {
      assigned_interceptor_id: "int-1",
      assigned_target_id: "tgt-1",
      last_intercept_pose: { x: 120, y: 30, z: 35 },
      threat_path_enu_m: [
        [200, 50, 40],
        [160, 40, 38],
        [120, 30, 35],
      ],
    } as TacticalStatePayload & { threat_path_enu_m: number[][] };
    const geom = deriveTacticalTrajectoryGeometry(state, entities);
    const threat = deriveThreatCorridorGeometry(state, geom!, entities);
    expect(threat?.mode).toBe("telemetry_path");
    expect(threat?.corridorPoints.length).toBeGreaterThanOrEqual(3);
  });

  it("uses bridge dict threat_path_enu_m when provided", () => {
    const state = {
      assigned_interceptor_id: "int-1",
      assigned_target_id: "tgt-1",
      last_intercept_pose: { x: 120, y: 30, z: 35 },
      threat_path_enu_m: [
        { x: 200, y: 50, z: 40 },
        { x: 160, y: 40, z: 38 },
        { x: 120, y: 30, z: 35 },
      ],
    } as TacticalStatePayload & {
      threat_path_enu_m: { x: number; y: number; z: number }[];
    };
    const geom = deriveTacticalTrajectoryGeometry(state, entities);
    const threat = deriveThreatCorridorGeometry(state, geom!, entities);
    expect(threat?.mode).toBe("telemetry_path");
    expect(threat?.corridorPoints[0]).toEqual({ x: 200, y: 50, z: 40 });
  });

  it("builds 7km direct_fallback corridor from derived intercept endpoint", () => {
    const longRangeEntities: MirrorEntity[] = [
      {
        entity_id: "int-1",
        entity_type: "interceptor",
        pose: { x: 0, y: 0, z: 20 },
      },
      {
        entity_id: "tgt-1",
        entity_type: "drone",
        pose: { x: 6000, y: 4000, z: 40 },
      },
    ];
    const state = {
      assigned_interceptor_id: "int-1",
      assigned_target_id: "tgt-1",
      predicted_path_enu_m: [
        { x: 0, y: 0, z: 20 },
        { x: 6000, y: 4000, z: 35 },
      ],
    } as TacticalStatePayload & {
      predicted_path_enu_m: { x: number; y: number; z: number }[];
    };
    const geom = deriveTacticalTrajectoryGeometry(state, longRangeEntities);
    const threat = deriveThreatCorridorGeometry(state, geom!, longRangeEntities);
    expect(threat?.mode).toBe("direct_fallback");
    expect(threat?.corridorPoints).toEqual([
      { x: 6000, y: 4000, z: 40 },
      { x: 6000, y: 4000, z: 35 },
    ]);
  });

  it("falls back to attacker → solution segment", () => {
    const state: TacticalStatePayload = {
      assigned_interceptor_id: "int-1",
      assigned_target_id: "tgt-1",
      last_intercept_pose: { x: 120, y: 30, z: 35 },
    };
    const geom = deriveTacticalTrajectoryGeometry(state, entities);
    const threat = deriveThreatCorridorGeometry(state, geom!, entities);
    expect(threat?.mode).toBe("direct_fallback");
    expect(threat?.corridorPoints).toEqual([
      { x: 200, y: 50, z: 40 },
      { x: 120, y: 30, z: 35 },
    ]);
  });

  it("builds a closed ribbon polygon", () => {
    const ribbon = buildCorridorRibbonPolygon(
      [
        { x: 0, y: 0, z: 10 },
        { x: 100, y: 0, z: 10 },
      ],
      10,
    );
    expect(ribbon.length).toBe(4);
  });
});
