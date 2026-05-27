import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { GRID_HEIGHT, GRID_WIDTH } from "@/world/gridCoords";
import { WorldEditingGrid } from "./WorldEditingGrid";

const CELL_SIZE = 14;

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

    const expectedWidth = GRID_WIDTH * CELL_SIZE;
    const expectedHeight = GRID_HEIGHT * CELL_SIZE;

    expect(expectedWidth).toBe(expectedHeight);
    expect(markup).toContain(`width="${expectedWidth}"`);
    expect(markup).toContain(`height="${expectedHeight}"`);
    expect(markup).toContain(`viewBox="0 0 ${expectedWidth} ${expectedHeight}"`);
    expect(markup).toContain(`width="${CELL_SIZE}" height="${CELL_SIZE}"`);
  });
});
