import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { MirrorEntity } from "./entityMarkers";
import { hasTacticalCompareGeometry } from "./tacticalCompareOverlay";

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
});
