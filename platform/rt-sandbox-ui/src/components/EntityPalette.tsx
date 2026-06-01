import { PackagePlus } from "lucide-react";
import { PanelShell } from "@/components/GovernanceChrome";
import type { EntityType } from "@/world/entityCatalog";
import { ENTITY_GLYPHS, ENTITY_LABELS, ENTITY_TYPES } from "@/world/entityCatalog";
import { ENTITY_TYPE_LIMITS } from "@/world/bounds";

export function EntityPalette({
  selectedType,
  onSelectType,
  entityCountsByType,
  editingEnabled,
}: {
  selectedType: EntityType;
  onSelectType: (t: EntityType) => void;
  entityCountsByType: Partial<Record<EntityType, number>>;
  editingEnabled: boolean;
}) {

  return (
    <PanelShell title="Entity palette" icon={PackagePlus} className="!p-3 [&>h2]:mb-2">
      <p className="mb-2 text-xs text-slate-500">
        Select type, then place on SVG grid or Cesium globe (same session registry commands).
      </p>
      <div className="flex flex-wrap gap-1.5">
        {ENTITY_TYPES.map((type) => {
          const count = String(entityCountsByType[type] || 0) + "/" + String(ENTITY_TYPE_LIMITS[type]);
          const active = selectedType === type;
          return (
            <button
              key={type}
              type="button"
              disabled={!editingEnabled}
              onClick={() => onSelectType(type)}
              className={`rounded border px-2.5 py-1 text-xs font-mono ${
                active
                  ? "border-emerald-500 bg-emerald-900/50 text-emerald-100"
                  : "border-slate-600 bg-slate-800 text-slate-300 hover:bg-slate-700"
              } disabled:opacity-40`}
            >
              {ENTITY_GLYPHS[type]} {ENTITY_LABELS[type]} ({count})
            </button>
          );
        })}
      </div>
    </PanelShell>
  );
}
