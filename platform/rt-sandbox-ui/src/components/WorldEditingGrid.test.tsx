import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import { GRID_HEIGHT, GRID_WIDTH } from "@/world/gridCoords";
import { WorldEditingGrid } from "./WorldEditingGrid";
import { WORLD_EDITOR_CELL_SIZE } from "./worldEditorLayout";

const baseProps = {
  entities: [
    {
      entity_id: "center-a",
      entity_type: "waypoint_marker" as const,
      pose: { x: 0, y: 0, z: 5 },
    },
  ],
  selectedEntityId: "center-a",
  selectedType: "waypoint_marker" as const,
  editingEnabled: true,
  worldSummary: undefined,
  onSelectEntity: () => undefined,
  onSpawn: () => undefined,
  onMove: () => undefined,
  onDelete: () => undefined,
};

describe("WorldEditingGrid", () => {
  it("renders a square grid surface with square cells", () => {
    const markup = renderToStaticMarkup(
      <WorldEditingGrid
        entities={[]}
        selectedEntityId={null}
        selectedType="drone"
        editingEnabled
        worldSummary={undefined}
        onSelectEntity={() => undefined}
        onSpawn={() => undefined}
        onMove={() => undefined}
        onDelete={() => undefined}
      />,
    );

    const expectedSize = GRID_WIDTH * WORLD_EDITOR_CELL_SIZE;

    expect(expectedSize).toBe(GRID_HEIGHT * WORLD_EDITOR_CELL_SIZE);
    expect(markup).toContain(`viewBox="0 0 ${expectedSize} ${expectedSize}"`);
    expect(markup).toContain(`width="${WORLD_EDITOR_CELL_SIZE}" height="${WORLD_EDITOR_CELL_SIZE}"`);
    expect(markup).toContain('data-testid="world-editor-grid-stage"');
    expect(markup).toContain('data-testid="world-editor-grid-section"');
    expect(markup).toContain('data-testid="core-grid-local-notice"');
    expect(markup).toContain("Legacy core grid");
    expect(markup).toContain("shrink-0");
    expect(markup).toContain("min-h-");
  });

  it("includes coordinate readout chrome", () => {
    const markup = renderToStaticMarkup(
      <WorldEditingGrid
        entities={[]}
        selectedEntityId={null}
        selectedType="drone"
        editingEnabled
        worldSummary={undefined}
        onSelectEntity={() => undefined}
        onSpawn={() => undefined}
        onMove={() => undefined}
        onDelete={() => undefined}
      />,
    );
    expect(markup).toContain('data-testid="world-editor-coordinate-bar"');
    expect(markup).toContain('data-testid="world-editor-coord-x"');
    expect(markup).toContain('data-testid="world-editor-coord-y"');
    expect(markup).toContain('data-testid="world-editor-coord-cell"');
    expect(markup).toContain('data-testid="world-editor-spawn-hint"');
    expect(markup).toContain('data-testid="world-editor-helper-text"');
    expect(markup).toContain("zoom");
  });

  it("shows designate protected center button for selected entity", () => {
    const markup = renderToStaticMarkup(
      <WorldEditingGrid
        {...baseProps}
        onDesignateProtectedCenter={() => undefined}
        designateProtectedCenterDisabled={false}
      />,
    );
    expect(markup).toContain('data-testid="designate-protected-center"');
    expect(markup).toContain("Designate Protected Center");
  });

  it("disables designate button when already designated", () => {
    const markup = renderToStaticMarkup(
      <WorldEditingGrid
        {...baseProps}
        protectedCenterEntityId="center-a"
        onDesignateProtectedCenter={() => undefined}
        designateProtectedCenterDisabled={false}
      />,
    );
    const button =
      markup.match(/<button[^>]*data-testid="designate-protected-center"[^>]*>/)?.[0] ?? "";
    expect(button).toMatch(/\sdisabled(?:=""|(?=\s|>))/);
    expect(markup).toContain("Protected center");
  });

  it("hides designate button when editing disabled", () => {
    const markup = renderToStaticMarkup(
      <WorldEditingGrid
        {...baseProps}
        editingEnabled={false}
        onDesignateProtectedCenter={() => undefined}
        designateProtectedCenterDisabled
      />,
    );
    expect(markup).not.toContain('data-testid="designate-protected-center"');
  });
});

describe("WorldEditingGrid protected center designation", () => {
  it("invokes designate handler from selected entity bar", () => {
    const onDesignateProtectedCenter = vi.fn();
    const markup = renderToStaticMarkup(
      <WorldEditingGrid
        {...baseProps}
        onDesignateProtectedCenter={onDesignateProtectedCenter}
        designateProtectedCenterDisabled={false}
      />,
    );
    expect(markup).toContain("Designate Protected Center");
    onDesignateProtectedCenter();
    expect(onDesignateProtectedCenter).toHaveBeenCalledTimes(1);
  });
});
