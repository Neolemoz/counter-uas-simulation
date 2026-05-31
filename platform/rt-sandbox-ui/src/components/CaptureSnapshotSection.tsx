import type { LiveCaptureSummary } from "@/telemetry/captureSummary";
import {
  formatCaptureStartedShort,
  shortCaptureId,
} from "@/telemetry/captureSummary";

function SnapshotField({ label, value }: { label: string; value: string }) {
  return (
    <div className="min-w-[4.5rem]">
      <div className="text-[10px] uppercase tracking-wide text-slate-500">{label}</div>
      <div className="font-mono text-[11px] text-slate-200">{value}</div>
    </div>
  );
}

export function CaptureSnapshotSection({
  summary,
}: {
  summary: LiveCaptureSummary;
}) {
  return (
    <div className="mb-3 rounded border border-violet-900/40 bg-violet-950/20 px-3 py-2">
      <div className="text-[10px] font-semibold uppercase tracking-wide text-violet-200/90">
        Capture snapshot
      </div>
      <div className="mt-2 flex flex-wrap items-end gap-x-4 gap-y-2">
        <SnapshotField label="status" value={summary.captureStatus} />
        <SnapshotField label="frames" value={String(summary.framesCount)} />
        <SnapshotField label="entities" value={String(summary.entitiesCount)} />
        <SnapshotField label="capture id" value={shortCaptureId(summary.captureId)} />
        <SnapshotField
          label="started"
          value={formatCaptureStartedShort(summary.startedUtc)}
        />
      </div>
    </div>
  );
}
