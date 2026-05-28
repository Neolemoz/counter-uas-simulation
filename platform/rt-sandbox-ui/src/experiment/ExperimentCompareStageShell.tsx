import { compareModeLabel, type CompareModeId } from "./experimentUnifiedReview";

export function ExperimentCompareStageShell({
  compareMode,
  primaryManifestRef,
  secondaryManifestRef,
}: {
  compareMode: CompareModeId;
  primaryManifestRef: string | null;
  secondaryManifestRef: string | null;
}) {
  return (
    <div
      className="space-y-2 rounded border border-slate-800 bg-slate-950/40 p-2"
      data-testid="compare-stage-shell"
    >
      <p className="text-[10px] font-semibold uppercase tracking-wide text-slate-500">
        Compare stage (read-only shell)
      </p>
      <p className="text-xs text-slate-400">
        Mode: <span className="font-mono text-slate-300">{compareModeLabel(compareMode)}</span>
      </p>
      {primaryManifestRef && (
        <p className="truncate font-mono text-[10px] text-slate-500">
          Primary: {primaryManifestRef}
        </p>
      )}
      {secondaryManifestRef && (
        <p className="truncate font-mono text-[10px] text-slate-500">
          Secondary: {secondaryManifestRef}
        </p>
      )}
      <p className="text-[10px] text-slate-500">
        Frozen X1 compare and F5 extended/matrix panels remain below — mode wiring is P2.
      </p>
    </div>
  );
}
