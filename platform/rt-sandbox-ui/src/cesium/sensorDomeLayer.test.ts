import { describe, expect, it } from "vitest";
import { countEntitiesInNominalDome, horizontalDistanceM } from "./sensorDomeLayer";
import type { MirrorEntity } from "./entityMarkers";

describe("sensorDomeLayer", () => {
  it("computes horizontal distance", () => {
    expect(horizontalDistanceM(0, 0, 3, 4)).toBe(5);
  });

  it("counts entities inside nominal dome", () => {
    const entities: MirrorEntity[] = [
      { entity_id: "r1", entity_type: "radar", pose: { x: 0, y: 0, z: 10 } },
      { entity_id: "d1", entity_type: "drone", pose: { x: 50, y: 0, z: 10 } },
      { entity_id: "d2", entity_type: "drone", pose: { x: 400, y: 0, z: 10 } },
    ];
    expect(countEntitiesInNominalDome(0, 0, entities, 200)).toBe(1);
  });
});
