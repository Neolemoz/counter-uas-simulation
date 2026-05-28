import { formatCompareStatus, type CompareStatusId } from "./compareStatusVocabulary";

const STATUS_CLASS: Record<CompareStatusId, string> = {
  aligned:
    "rounded border border-slate-700 bg-slate-900/80 px-1.5 py-0.5 text-[10px] text-slate-400",
  divergent:
    "rounded border border-slate-600 bg-slate-800 px-1.5 py-0.5 text-[10px] font-medium text-slate-300",
  missing:
    "rounded border border-slate-800 bg-slate-950/80 px-1.5 py-0.5 text-[10px] italic text-slate-500",
  explanatory:
    "rounded border border-slate-800 bg-slate-950/60 px-1.5 py-0.5 text-[10px] italic text-slate-500",
  not_comparable:
    "rounded border border-slate-700 bg-slate-900/80 px-1.5 py-0.5 text-[10px] text-slate-500",
};

export function CompareStatusChip({
  status,
  title,
}: {
  status: CompareStatusId;
  title?: string;
}) {
  return (
    <span
      className={STATUS_CLASS[status]}
      title={title}
      data-testid="compare-status-chip"
      data-status={status}
    >
      {formatCompareStatus(status)}
    </span>
  );
}
