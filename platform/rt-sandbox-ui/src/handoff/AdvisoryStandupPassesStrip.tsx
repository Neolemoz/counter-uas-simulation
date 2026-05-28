import { STANDUP_PASS_LABELS } from "./advisoryAggregationV2";

export function AdvisoryStandupPassesStrip() {
  return (
    <div className="mb-3 rounded border border-dashed border-slate-700 bg-slate-950/30 p-2">
      <h4 className="mb-1 text-[10px] font-semibold uppercase tracking-wide text-slate-500">
        Stand-up passes (maintainer discipline)
      </h4>
      <ul className="list-inside list-disc text-[10px] text-slate-500">
        {STANDUP_PASS_LABELS.map((label) => (
          <li key={label}>{label}</li>
        ))}
      </ul>
    </div>
  );
}
