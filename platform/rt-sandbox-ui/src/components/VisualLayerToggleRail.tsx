import {
  CANONICAL_VISUAL_LAYER_REGISTRY,
  densityBudgetSummary,
  groupLayersForUi,
  isLayerVisible,
  registryBudgetSummaryLine,
  type VisualLayerVisibility,
} from "@/cesium/visualLayerRegistry";

const TOGGLE_BUTTON_CLASS =
  "rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 hover:bg-slate-700 disabled:opacity-40";

export function VisualLayerToggleRail({
  visibility,
  onToggle,
  disabled = false,
  memoryLine,
}: {
  visibility: VisualLayerVisibility;
  onToggle: (layerId: string) => void;
  disabled?: boolean;
  memoryLine?: string;
}) {
  const groups = groupLayersForUi(CANONICAL_VISUAL_LAYER_REGISTRY);

  const budgetLine = registryBudgetSummaryLine(visibility);
  const density = densityBudgetSummary(visibility);

  return (
    <div className="flex flex-col gap-2">
      {groups.map((group) => (
        <div key={group.groupId} className="flex flex-wrap items-center gap-2">
          <span className="flex w-full items-center justify-between gap-2 text-[10px] font-medium uppercase tracking-wide text-slate-500">
            <span>{group.title}</span>
            <span className="text-slate-600">
              {group.layers.filter((layer) => isLayerVisible(visibility, layer)).length}/{group.layers.length}
            </span>
          </span>
          {group.layers.map((layer) => {
            const on = isLayerVisible(visibility, layer);
            return (
              <button
                key={layer.layer_id}
                type="button"
                className={TOGGLE_BUTTON_CLASS}
                disabled={disabled}
                title={layer.disclaimer || layer.label}
                onClick={() => onToggle(layer.layer_id)}
              >
                {layer.label}: {on ? "on" : "off"}
              </button>
            );
          })}
        </div>
      ))}
      <div className="space-y-1">
        <p
          className={`text-[10px] ${density.exceeded && density.densityWarningsEnabled ? "text-amber-300" : "text-slate-500"}`}
          data-testid="registry-budget-summary"
          title="Warn-only density summary - no layer is enforced or commanded"
        >
          {budgetLine}
        </p>
        {memoryLine && (
          <p className="text-[10px] text-slate-600" data-testid="layer-memory-summary">
            {memoryLine}
          </p>
        )}
      </div>
    </div>
  );
}
