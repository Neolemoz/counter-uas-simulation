import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { CesiumRuntimePanel } from "@/components/CesiumRuntimePanel";
import { CesiumSelectedEntityActionRow } from "@/components/CesiumSelectedEntityActionRow";
import { WorldEditingGrid } from "@/components/WorldEditingGrid";
import { GRID_HEIGHT, GRID_WIDTH } from "@/world/gridCoords";
import { WORLD_EDITOR_CELL_SIZE } from "@/components/worldEditorLayout";
import { defaultVisibilityFromRegistry } from "@/cesium/visualLayerRegistry";
import { ProtectedCenterStatusStrip } from "@/intelligence/ProtectedCenterStatusStrip";
import type { ProtectedCenterClearReason } from "@/intelligence/protectedCenterCopy";
import {
  PROTECTED_CENTER_CLEARED_RESET_COPY,
  PROTECTED_CENTER_REDESIGNATE_COPY,
} from "@/intelligence/protectedCenterCopy";

const entities = [
  {
    entity_id: "center-a",
    entity_type: "waypoint_marker" as const,
    pose: { x: 0, y: 0, z: 5 },
  },
];

function renderCrossSurface(props: {
  protectedCenterEntityId: string | null;
  protectedCenterRecoveryNotice?: ProtectedCenterClearReason | null;
  selectedEntityId?: string | null;
}) {
  const selectedEntityId = props.selectedEntityId ?? "center-a";
  const strip = renderToStaticMarkup(
    <ProtectedCenterStatusStrip
      protectedCenterEntityId={props.protectedCenterEntityId}
      protectedCenterRecoveryNotice={props.protectedCenterRecoveryNotice ?? null}
      entities={entities}
    />,
  );
  const grid = renderToStaticMarkup(
    <WorldEditingGrid
      entities={entities}
      selectedEntityId={selectedEntityId}
      selectedType="waypoint_marker"
      editingEnabled
      worldSummary={undefined}
      onSelectEntity={() => undefined}
      onSpawn={() => undefined}
      onMove={() => undefined}
      onDelete={() => undefined}
      onDesignateProtectedCenter={() => undefined}
      protectedCenterEntityId={props.protectedCenterEntityId}
      protectedCenterRecoveryNotice={props.protectedCenterRecoveryNotice ?? null}
      designateProtectedCenterDisabled={false}
    />,
  );
  const cesiumRow = renderToStaticMarkup(
    <CesiumSelectedEntityActionRow
      selectedEntity={entities[0]}
      protectedCenterEntityId={props.protectedCenterEntityId}
      onDesignateProtectedCenter={() => undefined}
      designateProtectedCenterDisabled={false}
    />,
  );
  const cesiumPanel = renderToStaticMarkup(
    <CesiumRuntimePanel
      sessionId="session-a"
      orderedSessionIds={["session-a"]}
      entities={entities}
      selectedEntityId={selectedEntityId}
      selectedType="waypoint_marker"
      worldSummary={undefined}
      mirrorSnapshot={undefined}
      pendingReconcile={false}
      editingEnabled
      onSelectEntity={() => undefined}
      onSpawn={() => undefined}
      onMove={() => undefined}
      onDelete={() => undefined}
      layerVisibility={defaultVisibilityFromRegistry()}
      onLayerVisibilityChange={() => undefined}
      protectedCenterEntityId={props.protectedCenterEntityId}
      protectedCenterRecoveryNotice={props.protectedCenterRecoveryNotice ?? null}
      onDesignateProtectedCenter={() => undefined}
      designateProtectedCenterDisabled={false}
    />,
  );
  return `${strip}${grid}${cesiumRow}${cesiumPanel}`;
}

describe("protected center continuity integration", () => {
  it("updates all surfaces after designation", () => {
    const markup = renderCrossSurface({ protectedCenterEntityId: "center-a" });
    expect(markup).toContain('data-testid="protected-center-designated"');
    expect(markup).toContain('data-testid="cesium-designate-protected-center"');
    expect(markup).toContain("Protected center");
    expect(markup).not.toContain('data-testid="protected-center-recovery-banner"');
  });

  it("shows recovery banner on all surfaces after reset clears designation", () => {
    const markup = renderCrossSurface({
      protectedCenterEntityId: null,
      protectedCenterRecoveryNotice: "reset_session",
    });
    expect(markup).toContain('data-testid="protected-center-none"');
    expect(markup).toContain('data-testid="protected-center-recovery-banner"');
    expect(markup).toContain('data-testid="world-editor-protected-center-recovery"');
    expect(markup).toContain('data-testid="cesium-protected-center-recovery"');
    expect(markup).toContain(PROTECTED_CENTER_CLEARED_RESET_COPY);
    expect(markup).toContain(PROTECTED_CENTER_REDESIGNATE_COPY);
    expect(markup).toContain("Designate Protected Center");
  });

  it("simulates designate then reset lifecycle", () => {
    const designated = renderCrossSurface({ protectedCenterEntityId: "center-a" });
    expect(designated).toContain("center-a");

    const afterReset = renderCrossSurface({
      protectedCenterEntityId: null,
      protectedCenterRecoveryNotice: "reset_session",
    });
    expect(afterReset).not.toContain('data-testid="protected-center-designated"');
    expect(afterReset).toContain('data-recovery-reason="reset_session"');
  });
});

describe("world editor grid sizing sanity", () => {
  it("keeps square grid dimensions for integration harness", () => {
    const expectedSize = GRID_WIDTH * WORLD_EDITOR_CELL_SIZE;
    expect(expectedSize).toBe(GRID_HEIGHT * WORLD_EDITOR_CELL_SIZE);
  });
});
