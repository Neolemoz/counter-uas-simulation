import { PanelShell } from "@/components/GovernanceChrome";
import { FidelityTruthCognitionStrip } from "@/components/FidelityTruthCognitionStrip";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import {
  cognitionSummary,
  formatAuthorityChip,
  sessionContextLine,
} from "@/telemetry/cognition";
import type { TerrainLayerVisibility } from "@/cesium/terrainLayers";
import { terrainHubSummary } from "@/cesium/terrainCognition";
import {
  fidelityHubLine,
  mergeFidelityContextFromPayloads,
} from "@/fidelity/fidelityCognition";
import { shortSessionId } from "@/workstation/sessionVisualIdentity";
import { StatusBadge } from "./StatusBadge";

const HUB_CHANNELS: { key: string; label: string }[] = [
  { key: "world_summary", label: "World summary" },
  { key: "session_health", label: "Session health" },
  { key: "entity_pose_mirror", label: "Entity pose mirror" },
];

function ChannelCognitionRow({
  label,
  snapshot,
}: {
  label: string;
  snapshot: ChannelSnapshot | undefined;
}) {
  if (!snapshot) {
    return (
      <div className="rounded border border-dashed border-slate-700 p-2 text-xs text-slate-500">
        <span className="font-semibold text-slate-400">{label}</span> — no snapshot
      </div>
    );
  }

  const c = cognitionSummary(snapshot.payload);

  return (
    <div
      className={`rounded border p-2 text-xs ${
        c.stale ? "border-amber-700/60 bg-amber-950/20" : "border-slate-700 bg-slate-950/40"
      }`}
    >
      <div className="mb-1 flex flex-wrap items-center gap-2">
        <span className="font-semibold text-slate-300">{label}</span>
        {c.stale && <StatusBadge label="stale" tone="warn" />}
        {c.badges.map((b) => (
          <StatusBadge
            key={b.label}
            label={b.label}
            tone={b.tone === "error" ? "error" : b.tone === "warn" ? "warn" : "ok"}
          />
        ))}
      </div>
      <p className="text-slate-300">
        <span className="font-medium text-slate-400">authority:</span>{" "}
        {formatAuthorityChip(c.authorityLabel)} — {c.authorityDescription}
      </p>
      <p className="text-slate-500">
        <span className="text-slate-600">source:</span> {c.source} — {c.sourceDescription}
      </p>
    </div>
  );
}

export function RuntimeCognitionHub({
  snapshots,
  sessionId,
  terrainLayers,
  terrainLayersEnabled = false,
  experimentCompareActive = false,
  experimentAnalyticsActive = false,
  experimentContinuityReviewActive = false,
  experimentF5Active = false,
}: {
  snapshots: {
    world_summary?: ChannelSnapshot;
    session_health?: ChannelSnapshot;
    entity_pose_mirror?: ChannelSnapshot;
  };
  sessionId?: string | null;
  terrainLayers?: TerrainLayerVisibility;
  terrainLayersEnabled?: boolean;
  experimentCompareActive?: boolean;
  experimentAnalyticsActive?: boolean;
  experimentContinuityReviewActive?: boolean;
  experimentF5Active?: boolean;
}) {
  const terrainLine = terrainHubSummary(terrainLayersEnabled, terrainLayers);
  const channelMap: Record<string, ChannelSnapshot | undefined> = {
    world_summary: snapshots.world_summary,
    session_health: snapshots.session_health,
    entity_pose_mirror: snapshots.entity_pose_mirror,
  };
  const fidelityContext = mergeFidelityContextFromPayloads(
    snapshots.world_summary?.payload as Record<string, unknown> | undefined,
    snapshots.session_health?.payload as Record<string, unknown> | undefined,
  );
  const fidelityLine = fidelityHubLine(fidelityContext);
  const worldSummaryPayload = snapshots.world_summary?.payload as
    | Record<string, unknown>
    | undefined;

  return (
    <PanelShell title="Runtime cognition hub">
      {sessionId && (
        <p className="mb-2 font-mono text-xs text-amber-200/80">
          {sessionContextLine(sessionId, "active") ?? `session ${shortSessionId(sessionId)}`}
        </p>
      )}
      <p className="mb-3 text-xs text-slate-500">
        Consolidated authority, source, and health from pull mirrors — explanatory only.
      </p>
      {terrainLine && (
        <p className="mb-3 text-xs text-emerald-400/90">{terrainLine}</p>
      )}
      {experimentCompareActive && (
        <p className="mb-3 text-xs text-sky-300/90">
          Experiment compare active — telemetry mirrors only; not operational A/B proof
        </p>
      )}
      {experimentAnalyticsActive && (
        <p className="mb-3 text-xs text-violet-300/90">
          Experiment analytics — derived summaries only
        </p>
      )}
      {experimentContinuityReviewActive && (
        <p className="mb-3 text-xs text-fuchsia-300/90">
          Tactical annex review — replay-boundary timelines only
        </p>
      )}
      {experimentF5Active && (
        <p className="mb-3 text-xs text-amber-300/90">
          Advanced experiment metrics — derived summaries only
        </p>
      )}
      <p className="mb-3 text-xs text-violet-300/90">{fidelityLine}</p>
      <FidelityTruthCognitionStrip
        fidelityContext={fidelityContext}
        worldSummary={worldSummaryPayload}
        showLosDivergence={terrainLayersEnabled}
        compact
      />
      <div className="space-y-2">
        {HUB_CHANNELS.map(({ key, label }) => (
          <ChannelCognitionRow
            key={key}
            label={label}
            snapshot={channelMap[key]}
          />
        ))}
      </div>
    </PanelShell>
  );
}
