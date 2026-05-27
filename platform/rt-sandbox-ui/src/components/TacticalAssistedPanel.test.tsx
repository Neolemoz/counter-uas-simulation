import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { TacticalAssistedPanel } from "./TacticalAssistedPanel";

describe("TacticalAssistedPanel", () => {
  it("shows recommendation approve and reject controls", () => {
    const markup = renderToStaticMarkup(
      <TacticalAssistedPanel
        sessionId="sess-1"
        editingEnabled
        entities={[
          {
            entity_id: "i1",
            entity_type: "interceptor",
            pose: { x: 0, y: 0, z: 10 },
          },
          {
            entity_id: "d1",
            entity_type: "drone",
            pose: { x: 100, y: 0, z: 10 },
          },
        ]}
        state={{ tactical_mode: "assisted" }}
        recommendation={{
          schema: "rt_tactical_recommendation_v1",
          recommendation_id: "rec-1",
          recommended_interceptor_id: "i1",
          recommended_target_id: "d1",
          tti_s: 3.1,
          feasibility: { feasible: true, reason: "feasible" },
          explanation: "Lowest cap-speed TTI",
          authority_label: "tactical_recommendation_explanatory",
        }}
        busy={false}
        error={null}
        onRequestRecommendation={() => undefined}
        onApprove={() => undefined}
        onReject={() => undefined}
      />,
    );
    expect(markup).toContain("Approve recommendation");
    expect(markup).toContain("Reject recommendation");
    expect(markup).toContain("Refresh recommendation");
    expect(markup).toContain("3.1");
    expect(markup).not.toContain("engage");
  });
});
