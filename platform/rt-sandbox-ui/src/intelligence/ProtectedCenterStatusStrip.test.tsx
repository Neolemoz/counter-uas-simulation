import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { ProtectedCenterStatusStrip } from "./ProtectedCenterStatusStrip";
import { PROTECTED_CENTER_NONE_COPY } from "./protectedCenterCopy";

describe("ProtectedCenterStatusStrip", () => {
  it("renders none designated state", () => {
    const markup = renderToStaticMarkup(
      <ProtectedCenterStatusStrip protectedCenterEntityId={null} entities={[]} />,
    );
    expect(markup).toContain('data-testid="protected-center-status-strip"');
    expect(markup).toContain('data-testid="protected-center-none"');
    expect(markup).toContain(PROTECTED_CENTER_NONE_COPY);
  });

  it("renders designated entity id and type", () => {
    const markup = renderToStaticMarkup(
      <ProtectedCenterStatusStrip
        protectedCenterEntityId="center-a"
        entities={[
          {
            entity_id: "center-a",
            entity_type: "radar",
            pose: { x: 0, y: 0, z: 10 },
          },
        ]}
      />,
    );
    expect(markup).toContain('data-testid="protected-center-designated"');
    expect(markup).toContain("center-a");
    expect(markup).toContain("Radar");
  });
});
