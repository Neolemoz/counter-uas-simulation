import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { MirrorEntity } from "./entityMarkers";
import { hasTacticalCompareGeometry } from "./tacticalCompareOverlay";
import {
  tacticalComparePathWidthPx,
  tacticalCompareSolutionPixelSize,
} from "./tacticalVisualScale";
import { WORLD_FIT_CAMERA_HEIGHT_M } from "@/world/bounds";
import { TIGHT_BOUNDS_CAMERA_HEIGHT_M } from "./visualStyle";

const entities: MirrorEntity[] = [
  {
    entity_id: "int-1",
    entity_type: "interceptor",
    pose: { x: 0, y: 0, z: 20 },
  },
  {
    entity_id: "tgt-1",
    entity_type: "drone",
    pose: { x: 120, y: 0, z: 30 },
  },
];

const path7km = [
  { x: 0, y: 0, z: 20 },
  { x: 6000, y: 4000, z: 35 },
];

describe("tacticalCompareOverlay", () => {
  it("detects compare geometry from intercept pose", () => {
    const state: TacticalStatePayload = {
      assigned_interceptor_id: "int-1",
      assigned_target_id: "tgt-1",
      last_intercept_pose: { x: 60, y: 0, z: 25 },
    };
    expect(hasTacticalCompareGeometry(state, entities)).toBe(true);
  });

  it("returns false when compare roles lack poses", () => {
    expect(
      hasTacticalCompareGeometry({ assigned_interceptor_id: "missing" }, entities),
    ).toBe(false);
  });

  it("scales compare path width and solution marker for long-range world-fit camera", () => {
    const cityCorePath = tacticalComparePathWidthPx(TIGHT_BOUNDS_CAMERA_HEIGHT_M, path7km);
    const worldFitPath = tacticalComparePathWidthPx(WORLD_FIT_CAMERA_HEIGHT_M, path7km);
    expect(worldFitPath).toBeGreaterThanOrEqual(cityCorePath);

    const cityCoreSolution = tacticalCompareSolutionPixelSize(
      TIGHT_BOUNDS_CAMERA_HEIGHT_M,
      7200,
    );
    const worldFitSolution = tacticalCompareSolutionPixelSize(
      WORLD_FIT_CAMERA_HEIGHT_M,
      7200,
    );
    expect(worldFitSolution).toBeGreaterThanOrEqual(cityCoreSolution);
  });
});
