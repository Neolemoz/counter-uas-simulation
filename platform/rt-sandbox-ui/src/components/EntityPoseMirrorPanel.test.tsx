import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { EntityPoseMirrorPanel } from "./EntityPoseMirrorPanel";

describe("EntityPoseMirrorPanel", () => {
  it("renders runtime telemetry rows for all entity types", () => {
    const markup = renderToStaticMarkup(
      <EntityPoseMirrorPanel
        snapshot={{
          channel: "entity_pose_mirror",
          timestamp_utc: "t0",
          payload: {
            entities: [
              {
                entity_id: "defender-alpha",
                entity_type: "interceptor",
                position: { x: 0, y: 0, z: 10 },
                heading_deg: 0,
                speed_mps: 0,
                assignment_state: "assigned",
                active_target_id: "attacker-alpha",
                target_state: "none",
              },
              {
                entity_id: "attacker-alpha",
                entity_type: "drone",
                pose: { x: 20, y: 0, z: 20 },
                heading_deg: 180,
                speed_mps: 12.5,
                target_state: "assigned",
              },
            ],
          },
        }}
        hideCognition
      />,
    );
    expect(markup).toContain("Runtime telemetry");
    expect(markup).toContain("defender-alpha");
    expect(markup).toContain("attacker-alpha");
    expect(markup).toContain("12.5 m/s");
    expect(markup).toContain("target ASSIGNED");
    expect(markup).toContain("assign ASSIGNED");
  });
});
