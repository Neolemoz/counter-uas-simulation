import { EntityRuntimeStatusChips } from "@/components/EntityRuntimeStatusChips";
import type { EntityRuntimeTelemetry } from "@/telemetry/entityMirrorFields";
import {
  formatActiveTargetId,
  formatHeadingDeg,
  formatPosition,
  formatSpeedMps,
} from "@/telemetry/entityMirrorFields";
import { ENTITY_GLYPHS, type EntityType } from "@/world/entityCatalog";

export function EntityRuntimeTelemetryRow({
  telemetry,
  typeLabel,
}: {
  telemetry: EntityRuntimeTelemetry;
  typeLabel: string;
}) {
  const glyph = ENTITY_GLYPHS[telemetry.entityType as EntityType] ?? "?";

  return (
    <div className="rounded border border-slate-800 bg-slate-950/50 px-2.5 py-2 text-xs">
      <div className="flex flex-wrap items-center gap-x-2 gap-y-1">
        <span className="font-mono text-emerald-200">{glyph}</span>
        <span className="font-semibold text-slate-200">{typeLabel}</span>
        <span className="font-mono text-[11px] text-slate-500">
          {telemetry.entityId.slice(0, 12)}
        </span>
      </div>
      <dl className="mt-2 grid grid-cols-2 gap-x-3 gap-y-1 font-mono text-[11px] text-slate-400 sm:grid-cols-3">
        <div>
          <dt className="text-[10px] uppercase tracking-wide text-slate-600">pos</dt>
          <dd className="text-slate-300">{formatPosition(telemetry.position)}</dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase tracking-wide text-slate-600">hdg</dt>
          <dd className="text-slate-300">{formatHeadingDeg(telemetry.headingDeg)}</dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase tracking-wide text-slate-600">spd</dt>
          <dd className="text-slate-300">{formatSpeedMps(telemetry.speedMps)}</dd>
        </div>
      </dl>
      <div className="mt-2">
        <EntityRuntimeStatusChips
          targetState={telemetry.targetState}
          assignmentState={telemetry.assignmentState}
        />
      </div>
      {telemetry.activeTargetId && (
        <p className="mt-1.5 font-mono text-[10px] text-slate-500">
          active target{" "}
          <span className="text-slate-300">
            {formatActiveTargetId(telemetry.activeTargetId)}
          </span>
        </p>
      )}
    </div>
  );
}
