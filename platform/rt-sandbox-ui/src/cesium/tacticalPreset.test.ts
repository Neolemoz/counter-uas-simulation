import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import {
  defaultVisibilityFromRegistry,
  type VisualLayerVisibility,
} from "./visualLayerRegistry";
import {
  enableTacticalViewPreset,
  isTacticalViewPresetActive,
  resolveTacticalTargetEntityId,
  TACTICAL_VIEW_GOVERNANCE_COPY,
  TACTICAL_VIEW_PRESET_LAYER_IDS,
} from "./tacticalPreset";

describe("tacticalPreset", () => {
  it("resolves tactical target entity id from assigned then selected state", () => {
    expect(
      resolveTacticalTargetEntityId({
        assigned_target_id: "assigned-target",
        selected_target_id: "selected-target",
      }),
    ).toBe("assigned-target");
    expect(
      resolveTacticalTargetEntityId({
        selected_target_id: "selected-target",
      }),
    ).toBe("selected-target");
    expect(resolveTacticalTargetEntityId(null)).toBeNull();
  });

  it("enables expected tactical preset layers without mutating input", () => {
    const base = defaultVisibilityFromRegistry();
    expect(base.showTacticalPredictedPath).toBe(false);

    const enabled = enableTacticalViewPreset(base);
    expect(enabled).not.toBe(base);
    expect(base.showTacticalPredictedPath).toBe(false);
    expect(enabled.showTacticalPredictedPath).toBe(true);
    expect(enabled.showTacticalInterceptPoint).toBe(true);
    expect(enabled.showTacticalTimingLabels).toBe(true);
    expect(enabled.showTacticalThreatCorridor).toBe(true);
    expect(enabled.showTacticalSelectionEmphasis).toBe(true);
    expect(enabled.showTacticalRankingCues).toBe(true);
    expect(enabled.showTacticalCompareOverlay).toBe(false);
    expect(isTacticalViewPresetActive(enabled)).toBe(true);
  });

  it("covers all preset layer ids in the bundle", () => {
    expect(TACTICAL_VIEW_PRESET_LAYER_IDS).toEqual([
      "tactical_predicted_path",
      "tactical_intercept_point",
      "tactical_timing_labels",
      "tactical_threat_corridor",
      "tactical_selection_emphasis",
      "tactical_ranking_cues",
    ]);
  });

  it("includes governance copy for visualization-only posture", () => {
    expect(TACTICAL_VIEW_GOVERNANCE_COPY).toMatch(/visualization only/i);
    expect(TACTICAL_VIEW_GOVERNANCE_COPY).toMatch(/no.*command/i);
    expect(TACTICAL_VIEW_GOVERNANCE_COPY).toMatch(/autonomous engagement/i);
  });

  it("does not mutate bridge tactical state objects", () => {
    const state: TacticalStatePayload = {
      assigned_target_id: "tgt-1",
      selected_target_id: "tgt-2",
      tactical_mode: "manual",
    };
    const snapshot = JSON.stringify(state);
    resolveTacticalTargetEntityId(state);
    expect(JSON.stringify(state)).toBe(snapshot);
  });

  it("preserves unrelated visibility keys when enabling preset", () => {
    const base: VisualLayerVisibility = {
      ...defaultVisibilityFromRegistry(),
      showLabels: true,
      showBounds: true,
    };
    const enabled = enableTacticalViewPreset(base);
    expect(enabled.showLabels).toBe(true);
    expect(enabled.showBounds).toBe(true);
  });
});
