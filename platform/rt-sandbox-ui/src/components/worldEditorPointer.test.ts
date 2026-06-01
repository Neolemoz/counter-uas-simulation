import { describe, expect, it } from "vitest";
import { cellCenterWorld, worldToCell } from "@/world/gridCoords";
import {
  clientToGridCell,
  clientToSvgPoint,
  meetContentRect,
  panOriginForGrab,
} from "./worldEditorPointer";

const CELL_SIZE = 14;
const GRID_PX = 40 * CELL_SIZE;

describe("worldEditorPointer", () => {
  it("computes letterboxed content rect for wide containers", () => {
    const content = meetContentRect(
      { left: 0, top: 0, width: 800, height: 400 },
      { viewBoxWidth: GRID_PX, viewBoxHeight: GRID_PX },
    );
    expect(content.width).toBe(400);
    expect(content.height).toBe(400);
    expect(content.left).toBe(200);
    expect(content.top).toBe(0);
  });

  it("maps pointer through letterboxing to the correct grid cell", () => {
    const view = { viewBoxX: 0, viewBoxY: 0, viewBoxWidth: GRID_PX, viewBoxHeight: GRID_PX };
    const clientRect = { left: 0, top: 0, width: 800, height: 400 };

    const topLeftCell = clientToGridCell(204, 4, clientRect, view, CELL_SIZE);
    expect(topLeftCell).toEqual({ col: 0, row: 0 });

    const centerPoint = clientToSvgPoint(400, 200, clientRect, view);
    expect(centerPoint?.x).toBeCloseTo(GRID_PX / 2, 5);
    expect(centerPoint?.y).toBeCloseTo(GRID_PX / 2, 5);
  });

  it("keeps hover/spawn world coords aligned with mapped cell center", () => {
    const view = { viewBoxX: 0, viewBoxY: 0, viewBoxWidth: GRID_PX, viewBoxHeight: GRID_PX };
    const clientRect = { left: 0, top: 0, width: 800, height: 400 };
    const cell = clientToGridCell(204, 4, clientRect, view, CELL_SIZE);
    expect(cell).not.toBeNull();
    const world = cellCenterWorld(cell!.col, cell!.row);
    expect(worldToCell(world.x, world.y)).toEqual(cell);
  });

  it("updates pan origin from grabbed viewBox point", () => {
    const clientRect = { left: 0, top: 0, width: 800, height: 400 };
    const centerSvg = GRID_PX / 2;
    const origin = panOriginForGrab(
      centerSvg,
      centerSvg,
      400,
      200,
      clientRect,
      GRID_PX,
      GRID_PX,
    );
    expect(origin.x).toBeCloseTo(0, 5);
    expect(origin.y).toBeCloseTo(0, 5);
  });
});
