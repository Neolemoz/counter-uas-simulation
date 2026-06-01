import {
  CANONICAL_VISUAL_LAYER_REGISTRY,
  densityBudgetSummary,
  groupLayersForUi,
  isLayerVisible,
  registryBudgetSummaryLine,
  type UiLayerGroup,
  type VisualLayerDescriptor,
  type VisualLayerVisibility,
} from "@/cesium/visualLayerRegistry";

const PRIMARY_LAYER_ORDER = ["terrain_mesh", "sensor_domes", "entity_labels"] as const;

const ADVANCED_GROUP_TITLES: Partial<Record<UiLayerGroup["groupId"], string>> = {
  terrain_context: "Terrain extras",
  visibility_context: "Visibility",
  density_context: "Density & budget",
  comparison_context: "Comparison",
  marker_context: "Bounds",
  sensor_context: "Sensor extras",
};

const TOGGLE_ON_CLASS =
  "rounded border border-cyan-700/60 bg-cyan-950/50 px-2 py-0.5 text-[11px] font-medium text-cyan-100 transition-colors duration-150 hover:bg-cyan-900/45 disabled:opacity-40";

const TOGGLE_OFF_CLASS =
  "rounded border border-slate-700 bg-slate-900/80 px-2 py-0.5 text-[11px] text-slate-400 transition-colors duration-150 hover:border-slate-600 hover:text-slate-200 disabled:opacity-40";

export function partitionLayersForCompactUi(
  groups: UiLayerGroup[],
): { primary: VisualLayerDescriptor[]; advanced: UiLayerGroup[] } {
  const primarySet = new Set<string>(PRIMARY_LAYER_ORDER);
  const primary: VisualLayerDescriptor[] = [];
  const advanced: UiLayerGroup[] = [];

  for (const group of groups) {
    const primaryInGroup = group.layers.filter((layer) => primarySet.has(layer.layer_id));
    const advancedInGroup = group.layers.filter((layer) => !primarySet.has(layer.layer_id));
    primary.push(...primaryInGroup);
    if (advancedInGroup.length > 0) {
      advanced.push({ ...group, layers: advancedInGroup });
    }
  }

  primary.sort(
    (a, b) =>
      PRIMARY_LAYER_ORDER.indexOf(a.layer_id as (typeof PRIMARY_LAYER_ORDER)[number]) -
      PRIMARY_LAYER_ORDER.indexOf(b.layer_id as (typeof PRIMARY_LAYER_ORDER)[number]),
  );

  return { primary, advanced };
}

function LayerToggleButton({
  layer,
  on,
  disabled,
  onToggle,
}: {
  layer: VisualLayerDescriptor;
  on: boolean;
  disabled: boolean;
  onToggle: (layerId: string) => void;
}) {
  return (
    <button
      type="button"
      className={on ? TOGGLE_ON_CLASS : TOGGLE_OFF_CLASS}
      disabled={disabled}
      title={layer.disclaimer || layer.label}
      onClick={() => onToggle(layer.layer_id)}
    >
      {layer.label}
    </button>
  );
}

export function VisualLayerToggleRail({
  visibility,
  onToggle,
  disabled = false,
  memoryLine,
  showAdvisoryFooter = true,
}: {
  visibility: VisualLayerVisibility;
  onToggle: (layerId: string) => void;
  disabled?: boolean;
  memoryLine?: string;
  /** When false, hides density/budget advisory footer even if layers are on. */
  showAdvisoryFooter?: boolean;
}) {
  const groups = groupLayersForUi(CANONICAL_VISUAL_LAYER_REGISTRY);
  const { primary, advanced } = partitionLayersForCompactUi(groups);

  const budgetLine = registryBudgetSummaryLine(visibility);
  const density = densityBudgetSummary(visibility);
  const advisoryVisible =
    showAdvisoryFooter &&
    (visibility.showLayerBudgetSummary || visibility.showDensityWarnings);

  return (
    <div className="flex max-h-[min(60vh,22rem)] flex-col gap-1.5 overflow-y-auto">
      <div className="flex flex-wrap gap-1">
        {primary.map((layer) => (
          <LayerToggleButton
            key={layer.layer_id}
            layer={layer}
            on={isLayerVisible(visibility, layer)}
            disabled={disabled}
            onToggle={onToggle}
          />
        ))}
      </div>

      {advanced.map((group) => (
        <details
          key={group.groupId}
          className="rounded border border-slate-800/70 bg-slate-950/40"
        >
          <summary className="cursor-pointer px-2 py-1 text-[10px] font-semibold uppercase tracking-wide text-slate-500 hover:text-slate-400">
            {ADVANCED_GROUP_TITLES[group.groupId] ?? group.title}
          </summary>
          <div className="flex flex-wrap gap-1 border-t border-slate-800/70 px-2 py-1.5">
            {group.layers.map((layer) => (
              <LayerToggleButton
                key={layer.layer_id}
                layer={layer}
                on={isLayerVisible(visibility, layer)}
                disabled={disabled}
                onToggle={onToggle}
              />
            ))}
          </div>
        </details>
      ))}

      {(advisoryVisible || memoryLine) && (
        <div className="space-y-0.5 border-t border-slate-800/70 pt-1.5">
          {advisoryVisible && (
            <p
              className={`text-[10px] leading-snug ${density.exceeded && density.densityWarningsEnabled ? "text-amber-300" : "text-slate-500"}`}
              data-testid="registry-budget-summary"
              title="Warn-only density summary - no layer is enforced or commanded"
            >
              {budgetLine}
            </p>
          )}
          {memoryLine && (
            <p className="text-[10px] leading-snug text-slate-600" data-testid="layer-memory-summary">
              {memoryLine}
            </p>
          )}
        </div>
      )}
    </div>
  );
}
