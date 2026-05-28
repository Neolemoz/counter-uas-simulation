import {
  primaryPresetForPass,
  STANDUP_PASSES,
  type StandupPassDef,
} from "./advisoryAggregationV2";
import type { FilterPresetId } from "./advisoryTypes";

export function AdvisoryStandupPassSelector({
  activePassId,
  onSelectPass,
  rowCountsByPass,
}: {
  activePassId: string | null;
  onSelectPass: (pass: StandupPassDef) => void;
  rowCountsByPass?: Partial<Record<string, number>>;
}) {
  return (
    <div className="mb-2">
      <p className="mb-1 text-[10px] text-amber-200/70">
        Pass selector ≠ CLI invocation — filters triage view only.
      </p>
      <div className="flex flex-wrap gap-1">
        {STANDUP_PASSES.map((pass) => {
          const active = activePassId === pass.pass_id;
          const count = rowCountsByPass?.[pass.pass_id];
          return (
            <button
              key={pass.pass_id}
              type="button"
              className={`rounded border px-2 py-0.5 text-[10px] ${
                active
                  ? "border-violet-700/60 bg-violet-950/40 text-violet-100"
                  : "border-slate-700 bg-slate-900 text-slate-400 hover:bg-slate-800"
              }`}
              onClick={() => onSelectPass(pass)}
              aria-pressed={active}
              title={
                pass.preset_ids.length
                  ? `Preset: ${pass.preset_ids.join(", ")}`
                  : pass.cohort_hint
                    ? `Cohort: ${pass.cohort_hint}`
                    : pass.label
              }
            >
              {pass.label}
              {count != null ? ` (${count})` : ""}
            </button>
          );
        })}
      </div>
    </div>
  );
}

export function filterPresetForStandupPass(pass: StandupPassDef): {
  preset: FilterPresetId;
  cohortHint: string | null;
} {
  const preset = primaryPresetForPass(pass);
  if (preset) return { preset, cohortHint: null };
  if (pass.cohort_hint) return { preset: "all_staged", cohortHint: pass.cohort_hint };
  return { preset: "all_staged", cohortHint: null };
}
