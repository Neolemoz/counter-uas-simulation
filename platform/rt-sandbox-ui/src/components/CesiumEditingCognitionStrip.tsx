import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { cognitionSummary, formatAuthorityChip } from "@/telemetry/cognition";
import {
  describeCommandIntent,
  describeMirrorLag,
  formatCommandResult,
} from "@/editing/cognition";
import type { EditCommandType } from "@/editing/editHistory";
import {
  cesiumEditingAuthorityNote,
  cesiumSyncAdapterNote,
} from "@/cesium/cognition";
import {
  adapterModeLabel,
  describeSyncSources,
  feedbackEntityRows,
  formatApplyLagMs,
  isAdapterSyncApplicable,
} from "@/sync/cognition";
import { StatusBadge } from "@/workstation/StatusBadge";

export function CesiumEditingCognitionStrip({
  lastCommand,
  pendingReconcile,
  mirrorSnapshot,
  worldSummary,
  editingEnabled,
}: {
  lastCommand?: {
    type: EditCommandType;
    ok: boolean;
    errorCode?: string;
    message?: string;
  };
  pendingReconcile: boolean;
  mirrorSnapshot: ChannelSnapshot | undefined;
  worldSummary: Record<string, unknown> | undefined;
  editingEnabled: boolean;
}) {
  const mirrorCognition = mirrorSnapshot
    ? cognitionSummary(mirrorSnapshot.payload)
    : null;
  const worldCognition = worldSummary
    ? cognitionSummary(worldSummary as Record<string, unknown>)
    : null;
  const driftRows = feedbackEntityRows(worldSummary);
  const lag = formatApplyLagMs(worldSummary);
  const adapterMode = adapterModeLabel(worldSummary);
  const mirrorStale = mirrorCognition?.stale ?? false;
  const worldStale = worldCognition?.stale ?? false;

  return (
    <div className="mt-3 space-y-3 rounded border border-sky-800/50 bg-sky-950/20 p-3">
      <p className="text-xs font-semibold uppercase text-sky-300/90">
        Cesium editing cognition
      </p>
      <p className="text-xs text-slate-500">{cesiumEditingAuthorityNote()}</p>
      <p className="text-xs text-slate-500">{cesiumSyncAdapterNote(worldSummary)}</p>
      <StatusBadge
        label={editingEnabled ? "globe editing enabled" : "globe editing blocked"}
        tone={editingEnabled ? "ok" : "neutral"}
      />
      <StatusBadge
        label={`adapter: ${adapterMode}`}
        tone={isAdapterSyncApplicable(worldSummary) ? "ok" : "neutral"}
      />
      {(mirrorStale || worldStale) && (
        <StatusBadge
          label="mirror stale"
          tone="warn"
          title="world_summary and/or entity_pose_mirror not ok"
        />
      )}
      {lastCommand && (
        <div className="text-xs text-slate-300">
          <p>{describeCommandIntent(lastCommand.type)}</p>
          <p className="text-slate-400">
            {formatCommandResult(
              lastCommand.ok,
              lastCommand.errorCode,
              lastCommand.message,
            )}
          </p>
        </div>
      )}
      <p className="text-xs italic text-slate-500">{describeMirrorLag(pendingReconcile)}</p>
      {lag && <p className="text-xs text-slate-400">Apply lag: {lag}</p>}
      {typeof worldSummary?.last_poll_utc === "string" && (
        <p className="text-xs text-slate-500">
          last poll: {String(worldSummary.last_poll_utc)}
        </p>
      )}
      <div className="text-xs">
        <p className="font-semibold text-slate-400">Sync sources</p>
        <ul className="mt-1 list-inside list-disc text-slate-500">
          {describeSyncSources().map((s) => (
            <li key={s.label}>
              {s.title}: {s.detail}
            </li>
          ))}
        </ul>
      </div>
      {worldCognition && (
        <div className="text-xs">
          <p className="font-semibold text-slate-400">World summary</p>
          <p className="text-slate-300">
            authority: {formatAuthorityChip(worldCognition.authorityLabel)} —{" "}
            {worldCognition.authorityDescription}
          </p>
          <div className="mt-1 flex flex-wrap gap-1">
            {worldCognition.badges.map((b) => (
              <StatusBadge
                key={b.label}
                label={b.label}
                tone={
                  b.tone === "error" ? "error" : b.tone === "warn" ? "warn" : "ok"
                }
              />
            ))}
          </div>
        </div>
      )}
      {driftRows.length > 0 && (
        <div className="text-xs">
          <p className="font-semibold text-slate-400">Per-entity drift</p>
          <ul className="mt-1 space-y-0.5 font-mono text-slate-500">
            {driftRows.map((row) => (
              <li key={row.entityId}>
                {row.entityId.slice(0, 8)}… drift=
                {row.driftM !== null ? row.driftM.toFixed(3) : "—"} m
              </li>
            ))}
          </ul>
        </div>
      )}
      {mirrorCognition && (
        <div className="text-xs">
          <p className="font-semibold text-slate-400">Pose mirror</p>
          <p className="text-slate-500">
            source: {mirrorCognition.source} — {mirrorCognition.sourceDescription}
          </p>
          {mirrorCognition.stale && (
            <StatusBadge label="stale mirror" tone="warn" />
          )}
        </div>
      )}
    </div>
  );
}
