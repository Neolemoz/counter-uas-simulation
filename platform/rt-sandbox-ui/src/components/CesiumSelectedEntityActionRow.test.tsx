import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import { CesiumSelectedEntityActionRow } from "./CesiumSelectedEntityActionRow";
import { ProtectedCenterStatusStrip } from "@/intelligence/ProtectedCenterStatusStrip";

const selectedEntity = {
  entity_id: "center-a",
  entity_type: "waypoint_marker" as const,
  pose: { x: 0, y: 0, z: 5 },
};

describe("CesiumSelectedEntityActionRow", () => {
  it("shows designate protected center action for selected entity", () => {
    const markup = renderToStaticMarkup(
      <CesiumSelectedEntityActionRow
        selectedEntity={selectedEntity}
        onDesignateProtectedCenter={() => undefined}
        designateProtectedCenterDisabled={false}
      />,
    );
    expect(markup).toContain('data-testid="cesium-selected-entity-actions"');
    expect(markup).toContain('data-testid="cesium-designate-protected-center"');
    expect(markup).toContain("Designate Protected Center");
  });

  it("disables designate button when entity is already designated", () => {
    const markup = renderToStaticMarkup(
      <CesiumSelectedEntityActionRow
        selectedEntity={selectedEntity}
        protectedCenterEntityId="center-a"
        onDesignateProtectedCenter={() => undefined}
        designateProtectedCenterDisabled={false}
      />,
    );
    const button =
      markup.match(
        /<button[^>]*data-testid="cesium-designate-protected-center"[^>]*>/,
      )?.[0] ?? "";
    expect(button).toMatch(/\sdisabled(?:=""|(?=\s|>))/);
    expect(markup).toContain("Protected center");
  });
});

describe("protected center status consistency", () => {
  it("uses the same protectedCenterEntityId for strip and cesium action row", () => {
    const entities = [selectedEntity];
    const stripMarkup = renderToStaticMarkup(
      <ProtectedCenterStatusStrip
        protectedCenterEntityId="center-a"
        entities={entities}
      />,
    );
    const cesiumMarkup = renderToStaticMarkup(
      <CesiumSelectedEntityActionRow
        selectedEntity={selectedEntity}
        protectedCenterEntityId="center-a"
        onDesignateProtectedCenter={() => undefined}
        designateProtectedCenterDisabled={false}
      />,
    );
    expect(stripMarkup).toContain("center-a");
    expect(cesiumMarkup).toContain("Protected center");
  });
});
