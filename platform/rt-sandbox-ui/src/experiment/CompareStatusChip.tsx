import { formatCompareStatus, type CompareStatusId } from "./compareStatusVocabulary";

export function CompareStatusChip({
  status,
  title,
}: {
  status: CompareStatusId;
  title?: string;
}) {
  const divergent = status === "divergent";
  return (
    <span
      className={
        divergent
          ? "rounded border border-slate-600 bg-slate-800 px-1.5 py-0.5 text-[10px] font-medium text-slate-300"
          : "rounded border border-slate-700 bg-slate-900/80 px-1.5 py-0.5 text-[10px] text-slate-400"
      }
      title={title}
      data-testid="compare-status-chip"
      data-status={status}
    >
      {formatCompareStatus(status)}
    </span>
  );
}
