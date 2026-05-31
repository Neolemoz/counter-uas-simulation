import { EntityRuntimeStatusChips } from "@/components/EntityRuntimeStatusChips";
import type { EntityRuntimeTelemetry } from "@/telemetry/entityMirrorFields";
import {
  formatActiveTargetId,
  formatHeadingDeg,
  formatSpeedMps,
} from "@/telemetry/entityMirrorFields";

function MetricPill({ label, value }: { label: string; value: string }) {
  return (
    <span className="inline-flex items-center gap-1 rounded border border-slate-700 bg-slate-950/80 px-2 py-0.5 font-mono text-[11px] text-slate-300">
      <span className="text-[10px] uppercase tracking-wide text-slate-500">{label}</span>
      <span>{value}</span>
    </span>
  );
}

export function SelectedEntityRuntimeStrip({
  telemetry,
}: {
  telemetry: EntityRuntimeTelemetry;
}) {
  return (
    <div className="mt-2 space-y-2 border-t border-amber-900/40 pt-2">
      <div className="flex flex-wrap items-center gap-1.5">
        <MetricPill label="hdg" value={formatHeadingDeg(telemetry.headingDeg)} />
        <MetricPill label="spd" value={formatSpeedMps(telemetry.speedMps)} />
        <EntityRuntimeStatusChips
          targetState={telemetry.targetState}
          assignmentState={telemetry.assignmentState}
        />
      </div>
      {telemetry.activeTargetId && (
        <p className="font-mono text-[11px] text-slate-400">
          <span className="text-[10px] uppercase tracking-wide text-slate-500">
            active target
          </span>{" "}
          <span className="text-slate-200">
            {formatActiveTargetId(telemetry.activeTargetId)}
          </span>
        </p>
      )}
    </div>
  );
}
