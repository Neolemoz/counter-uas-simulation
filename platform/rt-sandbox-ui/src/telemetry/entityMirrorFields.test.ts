import { describe, expect, it } from "vitest";
import {
  compactSelectedLabelSuffix,
  findEntityRuntimeTelemetry,
  parseEntityRuntimeTelemetry,
} from "./entityMirrorFields";

describe("entityMirrorFields", () => {
  it("parses runtime telemetry from mirror row", () => {
    const parsed = parseEntityRuntimeTelemetry({
      entity_id: "defender-alpha",
      entity_type: "interceptor",
      position: { x: 1, y: 2, z: 10 },
      heading_deg: 45,
      speed_mps: 12.3,
      target_state: "none",
      assignment_state: "assigned",
      active_target_id: "attacker-alpha",
    });
    expect(parsed.entityId).toBe("defender-alpha");
    expect(parsed.position).toEqual({ x: 1, y: 2, z: 10 });
    expect(parsed.headingDeg).toBe(45);
    expect(parsed.speedMps).toBe(12.3);
    expect(parsed.targetState).toBe("none");
    expect(parsed.assignmentState).toBe("assigned");
    expect(parsed.activeTargetId).toBe("attacker-alpha");
  });

  it("falls back to pose yaw and velocity speed", () => {
    const parsed = parseEntityRuntimeTelemetry({
      entity_id: "drone-1",
      entity_type: "drone",
      pose: { x: 0, y: 0, z: 5, yaw_deg: 180 },
      velocity: { speed_mps: 7.5, x: 1, y: 0, z: 0 },
      target_state: "assigned",
    });
    expect(parsed.headingDeg).toBe(180);
    expect(parsed.speedMps).toBe(7.5);
    expect(parsed.targetState).toBe("assigned");
  });

  it("finds telemetry by entity id", () => {
    const found = findEntityRuntimeTelemetry(
      [{ entity_id: "a", entity_type: "drone", speed_mps: 1 }],
      "a",
    );
    expect(found?.speedMps).toBe(1);
  });

  it("builds compact selected label suffix", () => {
    expect(
      compactSelectedLabelSuffix({
        entityId: "a",
        entityType: "drone",
        position: null,
        headingDeg: 90,
        speedMps: 4.2,
        targetState: "assigned",
        assignmentState: null,
        activeTargetId: null,
      }),
    ).toBe(" · hdg 90° spd 4.2 TGT");
  });
});
