import type { FilterPresetId } from "./advisoryTypes";
import { FILTER_PRESET_OPTIONS } from "./advisoryAggregationV2";
import { ADVISORY_GOVERNANCE_BANNER } from "./advisoryTypes";

export function AdvisoryPresetSelector({
  value,
  onChange,
}: {
  value: FilterPresetId;
  onChange: (id: FilterPresetId) => void;
}) {
  return (
    <div className="mb-3 rounded border border-slate-700 bg-slate-950/50 p-3">
      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Advisory filter preset (F8 — read-only)
      </h3>
      <p className="mb-2 text-[10px] text-amber-200/80">
        Filter preset ≠ CLI invocation. {ADVISORY_GOVERNANCE_BANNER}
      </p>
      <label className="flex flex-wrap items-center gap-2 text-xs text-slate-400">
        <span>Preset</span>
        <select
          className="rounded border border-slate-600 bg-slate-900 px-2 py-1 text-slate-200"
          value={value}
          onChange={(e) => onChange(e.target.value as FilterPresetId)}
          aria-label="Advisory filter preset"
        >
          {FILTER_PRESET_OPTIONS.map((o) => (
            <option key={o.id} value={o.id}>
              {o.label}
            </option>
          ))}
        </select>
      </label>
    </div>
  );
}
