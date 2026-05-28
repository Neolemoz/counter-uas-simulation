import type { ReactNode } from "react";
import { PanelShell } from "@/components/GovernanceChrome";
import { FidelityTruthCognitionStrip } from "@/components/FidelityTruthCognitionStrip";
import { VisibilityCognitionStrip } from "@/components/VisibilityCognitionStrip";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import {
  cognitionSummary,
  formatAuthorityChip,
  sessionContextLine,
} from "@/telemetry/cognition";
import type { MirrorEntity } from "@/cesium/entityMarkers";
import type { TerrainLayerVisibility } from "@/cesium/terrainLayers";
import { terrainHubSummary } from "@/cesium/terrainCognition";
import {
  sensorBlockVisible,
  sensorContextHubLine,
} from "@/cesium/visibilityCognition";
import {
  isFidelityCouplingOn,
  fidelityHubLine,
  mergeFidelityContextFromPayloads,
} from "@/fidelity/fidelityCognition";
import { shortSessionId } from "@/workstation/sessionVisualIdentity";
import {
  registryBudgetSummaryLine,
  type VisualLayerVisibility,
} from "@/cesium/visualLayerRegistry";
import { SessionComparisonCognitionStrip } from "./SessionComparisonCognitionStrip";
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

function CognitionBlock({
  title,
  defaultOpen,
  children,
}: {
  title: string;
  defaultOpen: boolean;
  children: ReactNode;
}) {
  return (
    <details
      className="rounded border border-slate-700 bg-slate-950/30"
      open={defaultOpen}
    >
      <summary className="cursor-pointer px-3 py-2 text-xs font-semibold text-slate-300">
        {title}
      </summary>
      <div className="border-t border-slate-800 px-3 py-2">{children}</div>
    </details>
  );
}

export function RuntimeCognitionHub({
  snapshots,
  sessionId,
  orderedSessionIds = [],
  layerVisibility,
  terrainLayers,
  terrainLayersEnabled = false,
  entities = [],
  selectedEntityId = null,
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
  orderedSessionIds?: readonly string[];
  layerVisibility: VisualLayerVisibility;
  terrainLayers?: TerrainLayerVisibility;
  terrainLayersEnabled?: boolean;
  entities?: MirrorEntity[];
  selectedEntityId?: string | null;
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
  const fidelityOn = isFidelityCouplingOn(fidelityContext);
  const worldSummaryPayload = snapshots.world_summary?.payload as
    | Record<string, unknown>
    | undefined;
  const selectedEntity =
    entities.find((e) => e.entity_id === selectedEntityId) ?? null;
  const sensorLine =
    terrainLayers != null ? sensorContextHubLine(terrainLayers, entities) : null;

  const budgetSummary = registryBudgetSummaryLine(layerVisibility);

  return (
    <PanelShell title="Runtime cognition">
      <CognitionBlock title="Session" defaultOpen>
        {sessionId && (
          <p className="mb-2 font-mono text-xs text-amber-200/80">
            {sessionContextLine(sessionId, "active") ??
              `session ${shortSessionId(sessionId)}`}
          </p>
        )}
        <p className="text-xs text-slate-500">
          Consolidated authority, source, and health from pull mirrors — explanatory only.
        </p>
        {experimentCompareActive && (
          <p className="mt-2 text-xs text-sky-300/90">
            Experiment compare active — telemetry mirrors only; not operational A/B proof
          </p>
        )}
        {experimentAnalyticsActive && (
          <p className="mt-2 text-xs text-violet-300/90">
            Experiment analytics — derived summaries only
          </p>
        )}
        {experimentContinuityReviewActive && (
          <p className="mt-2 text-xs text-fuchsia-300/90">
            Tactical annex review — replay-boundary timelines only
          </p>
        )}
        {experimentF5Active && (
          <p className="mt-2 text-xs text-amber-300/90">
            Advanced experiment metrics — derived summaries only
          </p>
        )}
      </CognitionBlock>

      <div className="mt-2 space-y-2">
        <CognitionBlock title="Terrain (explanatory)" defaultOpen={terrainLayersEnabled}>
          {terrainLine ? (
            <p className="text-xs text-emerald-400/90">{terrainLine}</p>
          ) : (
            <p className="text-xs text-slate-500">Terrain layers off.</p>
          )}
        </CognitionBlock>

        <CognitionBlock title="Density controls" defaultOpen={layerVisibility.showDensityWarnings}>
          <p className="text-xs text-slate-300" data-testid="hub-density-summary">
            {budgetSummary}
          </p>
          <p className="mt-1 text-[10px] text-slate-500">
            Warn-only display policy — registry and bridge command truth unchanged.
          </p>
        </CognitionBlock>

        <CognitionBlock
          title="Session compare (visual only)"
          defaultOpen={layerVisibility.showSessionContrast}
        >
          <SessionComparisonCognitionStrip
            activeSessionId={sessionId}
            orderedSessionIds={orderedSessionIds}
            comparisonGhostsEnabled={layerVisibility.showComparisonGhosts}
            sessionContrastEnabled={layerVisibility.showSessionContrast}
            compact
          />
          <p className="mt-1 text-[10px] text-slate-500">
            Compare surfaces are explanatory; only the selected session is commandable.
          </p>
        </CognitionBlock>

        <CognitionBlock title="Visibility (heuristic)" defaultOpen={false}>
          <p className="mb-2 text-[10px] text-slate-500" data-testid="hub-registry-budget">
            {budgetSummary}
          </p>
          <VisibilityCognitionStrip
            layerVisibility={layerVisibility}
            selectedEntity={selectedEntity}
            entities={entities}
          />
        </CognitionBlock>

        <CognitionBlock title="Fidelity (F5b)" defaultOpen={fidelityOn}>
          <p className="mb-2 text-xs text-violet-300/90">{fidelityLine}</p>
          <FidelityTruthCognitionStrip
            fidelityContext={fidelityContext}
            worldSummary={worldSummaryPayload}
            showLosDivergence={terrainLayersEnabled}
            compact
          />
        </CognitionBlock>

        <CognitionBlock
          title="Sensor context (nominal)"
          defaultOpen={terrainLayers != null && sensorBlockVisible(terrainLayers)}
        >
          {sensorLine ? (
            <p className="text-xs text-slate-300">{sensorLine}</p>
          ) : (
            <p className="text-xs text-slate-500">Sensor context layers off.</p>
          )}
        </CognitionBlock>

        <CognitionBlock title="Authority + channels" defaultOpen>
          <div className="space-y-2">
            {HUB_CHANNELS.map(({ key, label }) => (
              <ChannelCognitionRow
                key={key}
                label={label}
                snapshot={channelMap[key]}
              />
            ))}
          </div>
        </CognitionBlock>
      </div>
    </PanelShell>
  );
}
