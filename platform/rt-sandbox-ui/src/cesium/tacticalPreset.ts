import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import {
  layerIdToVisibilityKey,
  type VisualLayerVisibility,
} from "./visualLayerRegistry";
import { targetIdFromTacticalState } from "./tacticalSelectionEmphasisLayer";

export const TACTICAL_VIEW_GOVERNANCE_COPY =
  "TACTICAL VIEW — visualization only; overlays do not issue commands, assign intercepts, or enable autonomous engagement.";

export const TACTICAL_VIEW_PRESET_LAYER_IDS = [
  "tactical_predicted_path",
  "tactical_intercept_point",
  "tactical_timing_labels",
  "tactical_threat_corridor",
  "tactical_selection_emphasis",
  "tactical_ranking_cues",
] as const;

export type TacticalViewPresetLayerId = (typeof TACTICAL_VIEW_PRESET_LAYER_IDS)[number];

/** Display-only tactical target id for entity marker emphasis. */
export function resolveTacticalTargetEntityId(
  state: TacticalStatePayload | null | undefined,
): string | null {
  return targetIdFromTacticalState(state);
}

export function enableTacticalViewPreset(
  visibility: VisualLayerVisibility,
): VisualLayerVisibility {
  let next = { ...visibility };
  for (const layerId of TACTICAL_VIEW_PRESET_LAYER_IDS) {
    const key = layerIdToVisibilityKey(layerId);
    if (key) {
      next = { ...next, [key]: true };
    }
  }
  return next;
}

export function isTacticalViewPresetActive(
  visibility: VisualLayerVisibility,
): boolean {
  return TACTICAL_VIEW_PRESET_LAYER_IDS.every((layerId) => {
    const key = layerIdToVisibilityKey(layerId);
    return key ? visibility[key] === true : false;
  });
}

export function anyTacticalViewLayerActive(
  visibility: VisualLayerVisibility,
): boolean {
  return TACTICAL_VIEW_PRESET_LAYER_IDS.some((layerId) => {
    const key = layerIdToVisibilityKey(layerId);
    return key ? visibility[key] === true : false;
  });
}
