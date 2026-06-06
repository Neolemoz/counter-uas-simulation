import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { CesiumRuntimePanel } from "./CesiumRuntimePanel";
import { defaultVisibilityFromRegistry } from "@/cesium/visualLayerRegistry";

const baseProps = {
  sessionId: "session-a",
  orderedSessionIds: ["session-a"],
  entities: [
    {
      entity_id: "center-a",
      entity_type: "waypoint_marker" as const,
      pose: { x: 0, y: 0, z: 5 },
    },
  ],
  selectedEntityId: "center-a",
  selectedType: "waypoint_marker" as const,
  worldSummary: undefined,
  mirrorSnapshot: undefined,
  pendingReconcile: false,
  editingEnabled: true,
  onSelectEntity: () => undefined,
  onSpawn: () => undefined,
  onMove: () => undefined,
  onDelete: () => undefined,
  layerVisibility: defaultVisibilityFromRegistry(),
  onLayerVisibilityChange: () => undefined,
};

describe("CesiumRuntimePanel protected center designation", () => {
  it("renders cesium designate action when entity selected and handler provided", () => {
    const markup = renderToStaticMarkup(
      <CesiumRuntimePanel
        {...baseProps}
        onDesignateProtectedCenter={() => undefined}
        designateProtectedCenterDisabled={false}
      />,
    );
    expect(markup).toContain('data-testid="cesium-designate-protected-center"');
    expect(markup).toContain("Designate Protected Center");
  });

  it("hides cesium designate row when editing disabled", () => {
    const markup = renderToStaticMarkup(
      <CesiumRuntimePanel
        {...baseProps}
        editingEnabled={false}
        onDesignateProtectedCenter={() => undefined}
        designateProtectedCenterDisabled={false}
      />,
    );
    expect(markup).not.toContain('data-testid="cesium-selected-entity-actions"');
  });
});
