import type { MirrorEntity } from "@/cesium/entityMarkers";
import {
  anyVisibilityCognitionActive,
  visibilityActiveLabels,
  visibilityBudgetLine,
  visibilityLosLine,
  visibilityStripSummary,
  type VisibilityHubContext,
} from "@/cesium/visibilityCognition";
import type { TerrainLayerVisibility } from "@/cesium/terrainLayers";
import type { VisualLayerVisibility } from "@/cesium/visualLayerRegistry";

export function VisibilityCognitionStrip({
  layerVisibility,
  selectedEntity,
  entities,
  terrainLayers,
}: {
  layerVisibility: VisualLayerVisibility;
  selectedEntity: MirrorEntity | null;
  entities: MirrorEntity[];
  terrainLayers?: TerrainLayerVisibility;
}) {
  const active = visibilityActiveLabels(layerVisibility);
  const hubCtx: VisibilityHubContext = { selectedEntity, entities };
  const summary = visibilityStripSummary(layerVisibility, hubCtx, terrainLayers);
  const losLine = visibilityLosLine(
    selectedEntity,
    entities,
    layerVisibility.showStackedLos,
  );
  const budgetLine = visibilityBudgetLine(layerVisibility);

  if (!anyVisibilityCognitionActive(layerVisibility)) {
    return (
      <p className="text-xs text-slate-500">
        Visibility overlays off - enable wedge, horizon, stacked LOS, or V4 visibility cues for heuristic context.
      </p>
    );
  }

  return (
    <div className="space-y-1 text-xs text-slate-300">
      <p>
        <span className="font-medium text-slate-400">active: </span>
        {active.join(", ") || "none"}
      </p>
      <p className="text-slate-400">{summary}</p>
      {losLine && <p className="text-slate-400">{losLine}</p>}
      {budgetLine && <p className="text-amber-300/90">{budgetLine}</p>}
    </div>
  );
}
