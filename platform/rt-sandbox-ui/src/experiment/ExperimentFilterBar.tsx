import { BANNER_EXPERIMENT_F5 } from "@/governance/banners";
import {
  EMPTY_F5_FILTERS,
  F5_FILTER_ALL,
  type F5Filters,
} from "./experimentF5UiHelpers";

export function ExperimentFilterBar({
  filters,
  onChange,
  options,
}: {
  filters: F5Filters;
  onChange: (next: F5Filters) => void;
  options: {
    experiment_class: string[];
    tactical_mode: string[];
    terrain_preset: string[];
    visibility_context: string[];
  };
}) {
  const select = (
    label: string,
    key: keyof F5Filters,
    values: string[],
  ) => (
    <label className="text-xs text-slate-500">
      {label}
      <select
        className="ml-1 rounded border border-slate-700 bg-slate-950 px-1 py-0.5 text-xs"
        value={filters[key]}
        onChange={(e) => onChange({ ...filters, [key]: e.target.value })}
      >
        <option value={F5_FILTER_ALL}>all</option>
        {values.map((v) => (
          <option key={v} value={v}>
            {v}
          </option>
        ))}
      </select>
    </label>
  );

  return (
    <section className="space-y-2" data-testid="experiment-filter-bar">
      <p className="text-[10px] text-amber-100/80">{BANNER_EXPERIMENT_F5}</p>
      <p className="text-[10px] text-slate-500">
        Client-side manifest filters — explanatory only
      </p>
      <div className="flex flex-wrap gap-2">
        {select("class", "experiment_class", options.experiment_class)}
        {select("tactical", "tactical_mode", options.tactical_mode)}
        {select("terrain", "terrain_preset", options.terrain_preset)}
        {select("visibility", "visibility_context", options.visibility_context)}
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-0.5 text-xs text-slate-300"
          onClick={() => onChange(EMPTY_F5_FILTERS)}
        >
          Clear filters
        </button>
      </div>
    </section>
  );
}
