import type { MirrorEntity } from "@/cesium/entityMarkers";
import { ENTITY_GLYPHS, ENTITY_LABELS, type EntityType } from "@/world/entityCatalog";
import { DesignateProtectedCenterButton } from "@/components/DesignateProtectedCenterButton";

export function CesiumSelectedEntityActionRow({
  selectedEntity,
  protectedCenterEntityId = null,
  designateProtectedCenterDisabled = true,
  onDesignateProtectedCenter,
  onClearSelection,
}: {
  selectedEntity: MirrorEntity;
  protectedCenterEntityId?: string | null;
  designateProtectedCenterDisabled?: boolean;
  onDesignateProtectedCenter?: () => void;
  onClearSelection?: () => void;
}) {
  const entityType = selectedEntity.entity_type as EntityType;

  return (
    <div
      className="pointer-events-auto flex flex-wrap items-center justify-between gap-2 rounded border border-amber-500/50 bg-slate-950/90 px-3 py-2 shadow-lg shadow-black/40 backdrop-blur-sm"
      data-testid="cesium-selected-entity-actions"
    >
      <div className="min-w-0">
        <div className="text-[10px] font-semibold uppercase tracking-wide text-amber-100">
          Selected
        </div>
        <div className="truncate font-mono text-xs text-slate-200">
          {ENTITY_GLYPHS[entityType] ?? "?"} {ENTITY_LABELS[entityType] ?? entityType} ·{" "}
          {selectedEntity.entity_id.slice(0, 10)}
        </div>
      </div>
      <div className="flex shrink-0 flex-wrap gap-2">
        {onDesignateProtectedCenter && (
          <DesignateProtectedCenterButton
            selectedEntityId={selectedEntity.entity_id}
            protectedCenterEntityId={protectedCenterEntityId}
            disabled={designateProtectedCenterDisabled}
            onDesignate={onDesignateProtectedCenter}
            dataTestId="cesium-designate-protected-center"
          />
        )}
        {onClearSelection && (
          <button
            type="button"
            onClick={onClearSelection}
            className="rounded border border-slate-600 bg-slate-900 px-3 py-2 text-xs text-slate-200 hover:bg-slate-800"
          >
            Clear
          </button>
        )}
      </div>
    </div>
  );
}
