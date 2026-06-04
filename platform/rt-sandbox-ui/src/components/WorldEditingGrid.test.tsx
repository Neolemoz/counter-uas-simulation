import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { GRID_HEIGHT, GRID_WIDTH } from "@/world/gridCoords";
import { WorldEditingGrid } from "./WorldEditingGrid";
import { WORLD_EDITOR_CELL_SIZE } from "./worldEditorLayout";

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
});
