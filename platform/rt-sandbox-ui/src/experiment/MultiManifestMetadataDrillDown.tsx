import { CompareStatusChip } from "./CompareStatusChip";
import {
  MULTI_MANIFEST_DIFF_COLUMNS,
  sortMultiManifestRows,
  truncateManifestRef,
} from "./multiManifestDiffColumns";
import {
  MULTI_MANIFEST_DIFF_BANNER,
  type MultiManifestDiffRow,
  type ManifestSummaryChip,
} from "./multiManifestDiff";
import { ManifestSummaryChips } from "./ManifestSummaryChips";

export const DRILL_DOWN_TOOLTIP =
  "Metadata compare only — open manifest for run-level review";

export function MultiManifestMetadataDrillDown({
  rows,
  chips,
  primaryManifestRef,
  secondaryManifestRef,
  onOpenManifestRef,
}: {
  rows: MultiManifestDiffRow[];
  chips: ManifestSummaryChip[];
  primaryManifestRef: string;
  secondaryManifestRef: string;
  onOpenManifestRef?: (manifestRef: string, role: "primary" | "secondary") => void;
}) {
  const sortedRows = sortMultiManifestRows(rows);
  const primarySubtitle = truncateManifestRef(primaryManifestRef);
  const secondarySubtitle = truncateManifestRef(secondaryManifestRef);

  return (
    <div className="space-y-2" title={DRILL_DOWN_TOOLTIP}>
      <p className="text-[10px] text-amber-200/80">{MULTI_MANIFEST_DIFF_BANNER}</p>
      <p className="text-[10px] text-slate-500">{DRILL_DOWN_TOOLTIP}</p>
      <ManifestSummaryChips chips={chips} />
      {onOpenManifestRef && (
        <div className="flex flex-wrap gap-2">
          <button
            type="button"
            className="rounded border border-slate-600 px-2 py-0.5 text-[10px] text-cyan-400"
            onClick={() => onOpenManifestRef(primaryManifestRef, "primary")}
          >
            Open primary
          </button>
          <button
            type="button"
            className="rounded border border-slate-600 px-2 py-0.5 text-[10px] text-slate-400"
            onClick={() => onOpenManifestRef(secondaryManifestRef, "secondary")}
          >
            Open secondary
          </button>
        </div>
      )}
      <table className="w-full text-left text-xs text-slate-400">
        <thead>
          <tr className="border-b border-slate-800 text-[10px] uppercase text-slate-500">
            <th className="py-1 pr-2">{MULTI_MANIFEST_DIFF_COLUMNS[0].label}</th>
            <th className="py-1 pr-2">
              <span className="text-cyan-400/90">{MULTI_MANIFEST_DIFF_COLUMNS[1].label}</span>
              <span className="mt-0.5 block font-mono font-normal normal-case text-slate-600">
                {primarySubtitle}
              </span>
            </th>
            <th className="py-1 pr-2">
              <span>{MULTI_MANIFEST_DIFF_COLUMNS[2].label}</span>
              <span className="mt-0.5 block font-mono font-normal normal-case text-slate-600">
                {secondarySubtitle}
              </span>
            </th>
            <th className="py-1">{MULTI_MANIFEST_DIFF_COLUMNS[3].label}</th>
          </tr>
        </thead>
        <tbody>
          {sortedRows.map((row) => (
            <DiffRow key={row.field} row={row} />
          ))}
        </tbody>
      </table>
    </div>
  );
}

function DiffRow({ row }: { row: MultiManifestDiffRow }) {
  const { compare_status: status } = row;
  const divergent = status === "divergent";
  const missing = status === "missing";
  const cellMuted = missing ? "text-slate-600" : "text-slate-400";

  return (
    <tr
      className="border-b border-slate-900/80"
      data-testid={`multi-manifest-row-${row.field}`}
      data-compare-status={status}
    >
      <td
        className={`py-1 pr-2 font-mono text-slate-300 ${divergent ? "font-semibold" : ""}`}
        title={row.note ? `${row.source} — ${row.note}` : row.source}
      >
        {row.field}
      </td>
      <td className={`max-w-[8rem] truncate py-1 pr-2 ${cellMuted}`} title={row.primary}>
        {row.primary}
      </td>
      <td className={`max-w-[8rem] truncate py-1 pr-2 ${cellMuted}`} title={row.secondary}>
        {row.secondary}
      </td>
      <td className="py-1">
        <CompareStatusChip
          status={status}
          title={row.note ? `${row.source} — ${row.note}` : row.source}
        />
      </td>
    </tr>
  );
}
