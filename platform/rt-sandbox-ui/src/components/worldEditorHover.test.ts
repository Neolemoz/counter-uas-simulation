import { describe, expect, it } from "vitest";
import {
  cellsEqual,
  entityIdAtCell,
  shouldShowSpawnPreview,
} from "./worldEditorHover";

describe("worldEditorHover", () => {
  it("compares grid cells by col/row", () => {
    expect(cellsEqual({ col: 1, row: 2 }, { col: 1, row: 2 })).toBe(true);
    expect(cellsEqual({ col: 1, row: 2 }, { col: 1, row: 3 })).toBe(false);
  });

  it("resolves entity at hovered cell", () => {
    const entities = [
      { entity_id: "b", pose: { x: 0, y: 0 } },
      { entity_id: "a", pose: { x: 100, y: -100 } },
    ];
    expect(entityIdAtCell(entities, { col: 20, row: 20 })).toBe("b");
    expect(entityIdAtCell(entities, { col: 0, row: 0 })).toBeNull();
  });

  it("hides spawn preview when cell already has an entity", () => {
    expect(
      shouldShowSpawnPreview({
        editingEnabled: true,
        dragEntityId: null,
        panMoved: false,
        hoverCell: { col: 1, row: 1 },
        hoverEntityId: "e1",
        spawnSettleCell: null,
      }),
    ).toBe(false);
  });

  it("suppresses preview in settle cell after spawn click", () => {
    const cell = { col: 4, row: 5 };
    expect(
      shouldShowSpawnPreview({
        editingEnabled: true,
        dragEntityId: null,
        panMoved: false,
        hoverCell: cell,
        hoverEntityId: null,
        spawnSettleCell: cell,
      }),
    ).toBe(false);
  });
});
