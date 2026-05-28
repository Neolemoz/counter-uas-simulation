import type { ManifestSummaryChip } from "./multiManifestDiff";

export function ManifestSummaryChips({ chips }: { chips: ManifestSummaryChip[] }) {
  if (chips.length === 0) return null;
  return (
    <div className="flex flex-wrap gap-2" data-testid="manifest-summary-chips">
      {chips.map((chip) => (
        <div
          key={`${chip.role}-${chip.manifest_ref}`}
          className={
            chip.role === "primary"
              ? "rounded border border-cyan-800/50 bg-cyan-950/30 px-2 py-1"
              : "rounded border border-slate-700 bg-slate-900/60 px-2 py-1"
          }
        >
          <p className="text-[10px] font-semibold uppercase text-slate-500">
            {chip.role === "primary" ? "Primary" : "Secondary"}
          </p>
          <p className="font-mono text-xs text-slate-200">{chip.experiment_id}</p>
          <p
            className="truncate font-mono text-[10px] text-slate-500"
            title={chip.manifest_ref}
          >
            {chip.manifest_ref}
          </p>
          <p className="text-[10px] text-slate-400">
            {chip.runsLabel} · {chip.experiment_class}
          </p>
        </div>
      ))}
    </div>
  );
}
