import { ExperimentComparePanel } from "./ExperimentComparePanel";
import type { CompareSide } from "./experimentCompare";

export function ExperimentCompareSection({
  compareA,
  compareB,
  onCompareAChange,
  onCompareBChange,
  compareOptions,
  sideA,
  sideB,
  experimentId,
  compareMode,
}: {
  compareA: string;
  compareB: string;
  onCompareAChange: (value: string) => void;
  onCompareBChange: (value: string) => void;
  compareOptions: { value: string; label: string }[];
  sideA: CompareSide | null;
  sideB: CompareSide | null;
  experimentId: string;
  compareMode?: string;
}) {
  return (
    <>
      {compareMode === "multi_manifest_diff" && (
        <p className="mb-2 text-[10px] text-slate-500" data-testid="compare-metadata-drilldown-note">
          Metadata-only across manifests — use multi-manifest diff table above to open each
          manifest; run-level compare stays within one manifest.
        </p>
      )}
      <div className="mb-2 flex flex-wrap gap-2">
        <label className="text-xs text-slate-500">
          A
          <select
            className="ml-1 rounded border border-slate-700 bg-slate-950 px-1 py-0.5 text-xs"
            value={compareA}
            onChange={(e) => onCompareAChange(e.target.value)}
          >
            {compareOptions.map((o) => (
              <option key={o.value} value={o.value}>
                {o.label}
              </option>
            ))}
          </select>
        </label>
        <label className="text-xs text-slate-500">
          B
          <select
            className="ml-1 rounded border border-slate-700 bg-slate-950 px-1 py-0.5 text-xs"
            value={compareB}
            onChange={(e) => onCompareBChange(e.target.value)}
          >
            {compareOptions.map((o) => (
              <option key={o.value} value={o.value}>
                {o.label}
              </option>
            ))}
          </select>
        </label>
      </div>
      <ExperimentComparePanel sideA={sideA} sideB={sideB} experimentId={experimentId} />
    </>
  );
}
