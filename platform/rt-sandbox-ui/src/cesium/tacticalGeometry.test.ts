import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import {
  deriveDisplayInterceptPose,
  normalizeEnuPath,
  parseEnuPathPoint,
  parsePredictedPathTelemetry,
  parseThreatPathTelemetry,
} from "./tacticalGeometry";

describe("tacticalGeometry", () => {
  it("parses bridge dict path points", () => {
    expect(parseEnuPathPoint({ x: 0, y: 0, z: 20 })).toEqual({
      x: 0,
      y: 0,
      z: 20,
    });
    expect(
      normalizeEnuPath([
        { x: 0, y: 0, z: 20 },
        { x: 6000, y: 4000, z: 35 },
      ]),
    ).toEqual([
      { x: 0, y: 0, z: 20 },
      { x: 6000, y: 4000, z: 35 },
    ]);
  });

  it("parses legacy array path points", () => {
    expect(parseEnuPathPoint([120, 30, 35])).toEqual({ x: 120, y: 30, z: 35 });
    expect(
      normalizeEnuPath([
        [0, 0, 20],
        [60, 15, 28],
        [120, 30, 35],
      ]),
    ).toHaveLength(3);
  });

  it("derives intercept pose from predicted path endpoint", () => {
    const path = [
      { x: 0, y: 0, z: 20 },
      { x: 6000, y: 4000, z: 35 },
    ];
    const state = {
      predicted_path_enu_m: path,
    } as TacticalStatePayload;
    expect(deriveDisplayInterceptPose(state, path)).toEqual({
      x: 6000,
      y: 4000,
      z: 35,
    });
  });

  it("prefers last_intercept_pose over path endpoint", () => {
    const path = [
      { x: 0, y: 0, z: 20 },
      { x: 6000, y: 4000, z: 35 },
    ];
    const state = {
      last_intercept_pose: { x: 120, y: 30, z: 35 },
    } as TacticalStatePayload;
    expect(deriveDisplayInterceptPose(state, path)).toEqual({
      x: 120,
      y: 30,
      z: 35,
    });
  });

  it("reads predicted_path_enu_m from bridge dict telemetry", () => {
    const state = {
      predicted_path_enu_m: [
        { x: 0, y: 0, z: 10 },
        { x: 6000, y: 4000, z: 40 },
      ],
    } as TacticalStatePayload & {
      predicted_path_enu_m: { x: number; y: number; z: number }[];
    };
    expect(parsePredictedPathTelemetry(state)).toEqual([
      { x: 0, y: 0, z: 10 },
      { x: 6000, y: 4000, z: 40 },
    ]);
  });

  it("normalizes threat_path_enu_m in dict format", () => {
    const state = {
      threat_path_enu_m: [
        { x: 6000, y: 4000, z: 40 },
        { x: 3000, y: 2000, z: 38 },
        { x: 120, y: 30, z: 35 },
      ],
    } as TacticalStatePayload & {
      threat_path_enu_m: { x: number; y: number; z: number }[];
    };
    expect(parseThreatPathTelemetry(state)).toHaveLength(3);
  });
});
