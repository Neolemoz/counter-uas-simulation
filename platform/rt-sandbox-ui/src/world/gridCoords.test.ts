import { describe, expect, it } from "vitest";
import {
  cellCenterWorld,
  cellToWorld,
  entityCell,
  svgPointToCell,
  GRID_HEIGHT,
  GRID_WIDTH,
  worldToCell,
} from "./gridCoords";
import { clampPose } from "./bounds";

describe("gridCoords", () => {
  it("maps world origin to center cell", () => {
    const cell = worldToCell(0, 0);
    expect(cell.col).toBe(20);
    expect(cell.row).toBe(20);
    const corner = cellToWorld(cell.col, cell.row);
    expect(corner.x).toBe(0);
    expect(corner.y).toBe(0);
  });

  it("cell center within bounds after clamp", () => {
    const center = cellCenterWorld(20, 10);
    const clamped = clampPose({ x: center.x, y: center.y, z: 10 });
    expect(clamped.x).toBeGreaterThanOrEqual(-500);
    expect(clamped.x).toBeLessThanOrEqual(500);
  });

  it("entityCell matches worldToCell on pose", () => {
    const pose = { x: 100, y: -200 };
    expect(entityCell(pose)).toEqual(worldToCell(100, -200));
  });

  it("maps SVG top-left click to a top-left world cell", () => {
    const clickCell = svgPointToCell(1, 1, 14);
    const world = cellCenterWorld(clickCell.col, clickCell.row);

    expect(clickCell).toEqual({ col: 0, row: 0 });
    expect(worldToCell(world.x, world.y)).toEqual({ col: 0, row: 0 });
  });

  it("maps SVG bottom-left click to a bottom-left world cell", () => {
    const clickCell = svgPointToCell(1, GRID_HEIGHT * 14 - 1, 14);
    const world = cellCenterWorld(clickCell.col, clickCell.row);

    expect(clickCell).toEqual({ col: 0, row: GRID_HEIGHT - 1 });
    expect(worldToCell(world.x, world.y)).toEqual({ col: 0, row: GRID_HEIGHT - 1 });
  });

  it("maps SVG bottom-right click to a bottom-right world cell", () => {
    const clickCell = svgPointToCell(GRID_WIDTH * 14 - 1, GRID_HEIGHT * 14 - 1, 14);
    const world = cellCenterWorld(clickCell.col, clickCell.row);

    expect(clickCell).toEqual({ col: GRID_WIDTH - 1, row: GRID_HEIGHT - 1 });
    expect(worldToCell(world.x, world.y)).toEqual({
      col: GRID_WIDTH - 1,
      row: GRID_HEIGHT - 1,
    });
  });

  it("round-trips cell centers without changing visible row", () => {
    for (const row of [0, 10, GRID_HEIGHT - 1]) {
      const world = cellCenterWorld(5, row);
      expect(worldToCell(world.x, world.y)).toEqual({ col: 5, row });
    }
  });
});
