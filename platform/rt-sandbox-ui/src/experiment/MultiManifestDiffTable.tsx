import { formatCompareStatus } from "./compareStatusVocabulary";
import {
  buildMultiManifestDiff,
  buildManifestSummaryChips,
  MULTI_MANIFEST_DIFF_BANNER,
  type MultiManifestDiffRow,
} from "./multiManifestDiff";
import type { ExperimentCohortIndex } from "./cohortSchema";
import type { ExperimentManifest } from "./experimentSchema";
import { ManifestSummaryChips } from "./ManifestSummaryChips";

const DRILL_DOWN_TOOLTIP =
  "Metadata compare only — open manifest for run-level review";

export function MultiManifestDiffTable({
  cohort,
  primaryManifestRef,
  secondaryManifestRef,
  loadedManifest,
  onOpenManifestRef,
}: {
  cohort: ExperimentCohortIndex | null;
  primaryManifestRef: string | null;
  secondaryManifestRef: string | null;
  loadedManifest?: ExperimentManifest | null;
  onOpenManifestRef?: (manifestRef: string, role: "primary" | "secondary") => void;
}) {
  const rows = buildMultiManifestDiff({
    cohort,
    primaryManifestRef,
    secondaryManifestRef,
    loadedManifest,
  });

  const chips = buildManifestSummaryChips({
    cohort,
    primaryRef: primaryManifestRef,
    secondaryRef: secondaryManifestRef,
  });

  if (!primaryManifestRef || !secondaryManifestRef) {
    return (
      <div className="space-y-2" data-testid="multi-manifest-diff-table">
        <p className="text-[10px] text-amber-200/80">{MULTI_MANIFEST_DIFF_BANNER}</p>
        <p className="text-xs text-slate-500">
          Set primary and secondary manifests in the manifest roster.
        </p>
      </div>
    );
  }

  if (primaryManifestRef === secondaryManifestRef) {
    return (
      <div className="space-y-2" data-testid="multi-manifest-diff-table">
        <p className="text-[10px] text-amber-200/80">{MULTI_MANIFEST_DIFF_BANNER}</p>
        <p className="text-xs text-slate-500">Primary and secondary must be different manifests.</p>
      </div>
    );
  }

  if (rows.length === 0) {
    return (
      <div className="space-y-2" data-testid="multi-manifest-diff-table">
        <p className="text-[10px] text-amber-200/80">{MULTI_MANIFEST_DIFF_BANNER}</p>
        <p className="text-xs text-slate-500">
          Import a cohort index and select refs that exist in the active cohort.
        </p>
      </div>
    );
  }

  return (
    <div className="space-y-2" data-testid="multi-manifest-diff-table" title={DRILL_DOWN_TOOLTIP}>
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
            <th className="py-1 pr-2">Field</th>
            <th className="py-1 pr-2">Primary</th>
            <th className="py-1 pr-2">Secondary</th>
            <th className="py-1">Status</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((row) => (
            <DiffRow key={row.field} row={row} />
          ))}
        </tbody>
      </table>
    </div>
  );
}

function DiffRow({ row }: { row: MultiManifestDiffRow }) {
  const divergent = row.compare_status === "divergent";
  return (
    <tr className="border-b border-slate-900/80">
      <td
        className={`py-1 pr-2 font-mono text-slate-300 ${divergent ? "font-semibold" : ""}`}
        title={row.note ? `${row.source} — ${row.note}` : row.source}
      >
        {row.field}
      </td>
      <td className="max-w-[8rem] truncate py-1 pr-2" title={row.primary}>
        {row.primary}
      </td>
      <td className="max-w-[8rem] truncate py-1 pr-2" title={row.secondary}>
        {row.secondary}
      </td>
      <td className="py-1 text-[10px] text-slate-500">
        {formatCompareStatus(row.compare_status)}
      </td>
    </tr>
  );
}
