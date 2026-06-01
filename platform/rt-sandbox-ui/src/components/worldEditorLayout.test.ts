import { describe, expect, it } from "vitest";
import {
  formatWorldEditorCell,
  formatWorldEditorCoord,
  WORLD_EDITOR_CELL_SIZE,
  WORLD_EDITOR_COORD_BAR_HEIGHT_CLASS,
  WORLD_EDITOR_STAGE_MAX_PX,
  WORLD_EDITOR_STAGE_MIN_PX,
  worldEditorStageClassName,
} from "./worldEditorLayout";

describe("worldEditorLayout", () => {
  it("uses larger cells and stage than the prior baseline", () => {
    expect(WORLD_EDITOR_CELL_SIZE).toBeGreaterThanOrEqual(28);
    expect(WORLD_EDITOR_STAGE_MAX_PX).toBeGreaterThanOrEqual(900);
    expect(WORLD_EDITOR_STAGE_MIN_PX).toBeGreaterThanOrEqual(280);
  });

  it("pads world coordinates to a fixed width", () => {
    expect(formatWorldEditorCoord(12)).toHaveLength(5);
    expect(formatWorldEditorCoord(-480)).toHaveLength(5);
    expect(formatWorldEditorCoord(null)).toBe("———");
  });

  it("keeps cell labels stable when hover is absent", () => {
    expect(formatWorldEditorCell(null, null)).toBe("—,—");
    expect(formatWorldEditorCell(3, 7)).toBe("3,7");
  });

  it("keeps grid stage visible with full width and minimum height", () => {
    const stageClass = worldEditorStageClassName();
    expect(stageClass).toContain("w-full");
    expect(stageClass).toContain("shrink-0");
    expect(stageClass).toContain(String(WORLD_EDITOR_STAGE_MIN_PX));
    expect(stageClass).toContain(String(WORLD_EDITOR_STAGE_MAX_PX));
    expect(stageClass).toContain("aspect-square");
  });

  it("uses a fixed-height coordinate bar", () => {
    expect(WORLD_EDITOR_COORD_BAR_HEIGHT_CLASS).toContain("h-");
  });
});
