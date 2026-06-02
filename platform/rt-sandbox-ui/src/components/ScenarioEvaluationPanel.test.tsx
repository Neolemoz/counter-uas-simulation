import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ScenarioEvaluationPanel } from "./ScenarioEvaluationPanel";

describe("ScenarioEvaluationPanel", () => {
  it("renders scenario evaluation chrome and presets", () => {
    const markup = renderToStaticMarkup(
      <ScenarioEvaluationPanel
        sessionId="sess-1"
        entities={[
          {
            entity_id: "r1",
            entity_type: "radar",
            pose: { x: 0, y: 0, z: 10, yaw_deg: 0 },
          },
          {
            entity_id: "i1",
            entity_type: "interceptor",
            pose: { x: -5, y: 0, z: 10, yaw_deg: 0 },
          },
          {
            entity_id: "i2",
            entity_type: "interceptor",
            pose: { x: 4, y: -4, z: 10, yaw_deg: 0 },
          },
          {
            entity_id: "i3",
            entity_type: "interceptor",
            pose: { x: -4, y: 5, z: 10, yaw_deg: 0 },
          },
          {
            entity_id: "d1",
            entity_type: "drone",
            pose: { x: -1500, y: 0, z: 300, yaw_deg: 0 },
          },
        ]}
      />,
    );

    expect(markup).toContain("Scenario Evaluation");
    expect(markup).toContain("Generate MC Profile");
    expect(markup).toContain("Quick");
    expect(markup).toContain("Standard");
    expect(markup).toContain("Deep");
    expect(markup).toContain("MC Preview");
    expect(markup).toContain("Prepare MC Job");
    expect(markup).toContain("Prepared MC Job");
    expect(markup).toContain("Freeze-ready handoff");
    expect(markup).toContain("Copy layout JSON");
    expect(markup).toContain("Download bundle");
    expect(markup).toContain("Does not run Monte Carlo");
  });
});
