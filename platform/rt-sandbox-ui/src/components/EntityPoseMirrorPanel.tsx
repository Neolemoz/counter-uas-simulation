import { LocateFixed } from "lucide-react";
import { CaptureSnapshotSection } from "@/components/CaptureSnapshotSection";
import { EntityRuntimeTelemetryRow } from "@/components/EntityRuntimeTelemetryRow";
import { PanelShell } from "./GovernanceChrome";
import { TelemetryCognitionStrip } from "./TelemetryCognitionStrip";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { entitiesFromSnapshot } from "@/telemetry/channelIndex";
import type { LiveCaptureSummary } from "@/telemetry/captureSummary";
import { parseEntityRuntimeTelemetry } from "@/telemetry/entityMirrorFields";
import { ENTITY_GLYPHS, ENTITY_LABELS, type EntityType } from "@/world/entityCatalog";
import { entityCell, GRID_HEIGHT, GRID_WIDTH } from "@/world/gridCoords";

export function EntityPoseMirrorPanel({
  snapshot,
  hideCognition = false,
  sessionId = null,
  captureSummary,
}: {
  snapshot: ChannelSnapshot | undefined;
  hideCognition?: boolean;
  sessionId?: string | null;
  captureSummary?: LiveCaptureSummary;
}) {
  const entities = entitiesFromSnapshot(snapshot);

  return (
    <PanelShell title="Entity pose mirror" icon={LocateFixed} variant="tertiary">
      <p className="mb-3 text-xs leading-relaxed text-slate-500">
        Explanatory mirror snapshot — registry commands stay in the world editor.
      </p>
      {sessionId && captureSummary && (
        <CaptureSnapshotSection summary={captureSummary} />
      )}
      <div className="relative h-[220px] overflow-hidden rounded border border-slate-800 bg-slate-950/80 p-3">
        <div
          className="absolute inset-0 opacity-40"
          style={{
            backgroundImage:
              "linear-gradient(rgba(148,163,184,0.14) 1px, transparent 1px), linear-gradient(90deg, rgba(148,163,184,0.14) 1px, transparent 1px)",
            backgroundSize: "20px 20px",
          }}
          aria-hidden
        />
        {entities.length === 0 ? (
          <div className="relative flex h-full items-center justify-center rounded border border-dashed border-slate-800 text-center text-xs text-slate-500">
            No pose mirror entities in the latest pull.
          </div>
        ) : (
          <div className="relative h-full">
            {entities.map((entity, index) => {
              const telemetry = parseEntityRuntimeTelemetry(entity);
              const entityId = telemetry.entityId || `entity-${index}`;
              const entityType = telemetry.entityType || "unknown";
              const cell = entityCell(
                telemetry.position ?? (entity.pose as Record<string, unknown>) ?? {},
              );
              const glyph = ENTITY_GLYPHS[entityType as EntityType] ?? "?";
              const left = ((cell.col + 0.5) / GRID_WIDTH) * 100;
              const top = ((cell.row + 0.5) / GRID_HEIGHT) * 100;
              return (
                <div
                  key={entityId}
                  className="absolute flex h-6 w-6 -translate-x-1/2 -translate-y-1/2 items-center justify-center rounded border border-emerald-400/70 bg-emerald-950/90 font-mono text-[11px] text-emerald-100"
                  style={{ left: `${left}%`, top: `${top}%` }}
                  title={`${entityType} ${entityId}`}
                >
                  {glyph}
                </div>
              );
            })}
          </div>
        )}
      </div>
      {entities.length > 0 && (
        <div className="mt-3 space-y-2">
          <div className="text-[10px] font-semibold uppercase tracking-wide text-slate-500">
            Runtime telemetry
          </div>
          <div className="max-h-52 space-y-2 overflow-y-auto">
            {entities.map((entity, index) => {
              const telemetry = parseEntityRuntimeTelemetry(entity);
              const entityId = telemetry.entityId || `entity-${index}`;
              const typeLabel =
                ENTITY_LABELS[telemetry.entityType as EntityType] ??
                telemetry.entityType;
              return (
                <EntityRuntimeTelemetryRow
                  key={entityId}
                  telemetry={telemetry}
                  typeLabel={typeLabel}
                />
              );
            })}
          </div>
        </div>
      )}
      {!hideCognition && <TelemetryCognitionStrip snapshot={snapshot} />}
    </PanelShell>
  );
}
