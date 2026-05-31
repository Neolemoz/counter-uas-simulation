import type { LiveCaptureSummary } from "@/telemetry/captureSummary";
import {
  formatCaptureStartedShort,
  shortCaptureId,
} from "@/telemetry/captureSummary";

function SummaryItem({ label, value }: { label: string; value: string }) {
  return (
    <span className="inline-flex items-center gap-1">
      <span className="text-[10px] uppercase tracking-wide text-slate-500">{label}</span>
      <span className="text-slate-300">{value}</span>
    </span>
  );
}

export function CaptureSummaryStrip({
  summary,
}: {
  summary: LiveCaptureSummary;
}) {
  return (
    <div className="flex flex-wrap items-center gap-x-3 gap-y-1 rounded-lg border border-slate-700/80 bg-slate-900/50 px-3 py-2 font-mono text-[11px]">
      <SummaryItem
        label="status"
        value={summary.captureStatus}
      />
      <SummaryItem label="frames" value={String(summary.framesCount)} />
      <SummaryItem label="entities" value={String(summary.entitiesCount)} />
      <SummaryItem
        label="started"
        value={formatCaptureStartedShort(summary.startedUtc)}
      />
      <SummaryItem label="id" value={shortCaptureId(summary.captureId)} />
    </div>
  );
}
