import { describe, expect, it } from "vitest";
import { targetIdFromTacticalState } from "./tacticalSelectionEmphasisLayer";
import {
  tacticalSelectionHaloPixelSize,
  tacticalSelectionLabelOffset,
} from "./tacticalVisualScale";
import { WORLD_FIT_CAMERA_HEIGHT_M } from "@/world/bounds";
import { TIGHT_BOUNDS_CAMERA_HEIGHT_M } from "./visualStyle";
import { resolveTacticalTargetEntityId } from "./tacticalPreset";

describe("tacticalSelectionEmphasisLayer", () => {
  it("prefers assigned target over selected target for visual emphasis", () => {
    expect(
      targetIdFromTacticalState({
        assigned_target_id: "assigned-target",
        selected_target_id: "selected-target",
      }),
    ).toBe("assigned-target");
  });

  it("falls back to selected target when no assignment exists", () => {
    expect(targetIdFromTacticalState({ selected_target_id: "selected-target" })).toBe(
      "selected-target",
    );
  });

  it("matches entity marker tactical target resolver", () => {
    const state = {
      assigned_target_id: "assigned-target",
      selected_target_id: "selected-target",
    };
    expect(resolveTacticalTargetEntityId(state)).toBe(
      targetIdFromTacticalState(state),
    );
  });

  it("scales selection halo and label offset between city-core and world-fit camera", () => {
    const cityCoreHalo = tacticalSelectionHaloPixelSize(TIGHT_BOUNDS_CAMERA_HEIGHT_M);
    const worldFitHalo = tacticalSelectionHaloPixelSize(WORLD_FIT_CAMERA_HEIGHT_M);
    expect(worldFitHalo).toBeGreaterThanOrEqual(cityCoreHalo);

    const cityCoreLabel = tacticalSelectionLabelOffset(TIGHT_BOUNDS_CAMERA_HEIGHT_M);
    const worldFitLabel = tacticalSelectionLabelOffset(WORLD_FIT_CAMERA_HEIGHT_M);
    expect(Math.abs(worldFitLabel.y)).toBeGreaterThanOrEqual(Math.abs(cityCoreLabel.y));
  });
});
