import {
  Cartesian2,
  Color,
  Entity,
  LabelStyle,
  VerticalOrigin,
  Viewer,
} from "cesium";
import type {
  TacticalRecommendationPayload,
  TacticalStatePayload,
} from "@/bridge/tacticalCommands";
import { isViewerUsable } from "./cesiumEditing";
import { worldToCartesian } from "./coordinates";
import type { MirrorEntity } from "./entityMarkers";
import { applyTerrainDisplayOffset } from "./rtFictionalTerrain";
import {
  parseTacticalTargetRanking,
  rankLabelFor,
  type TacticalTargetRankEntry,
} from "./tacticalTargetRanking";
const TACTICAL_RANKING_PREFIX = "rt-tactical-ranking-";

export type TacticalRankingCueMode = "ranked_list" | "recommendation_only";

export interface TacticalRankingCueEntry extends TacticalTargetRankEntry {
  label: string;
}

export interface TacticalRankingCueResolution {
  mode: TacticalRankingCueMode;
  cues: TacticalRankingCueEntry[];
}

const RANKING_ARRAY_KEYS = [
  "ranked_candidates",
  "target_ranking",
  "target_candidates",
  "ranked_targets",
  "candidate_ranking",
  "threat_ranking",
] as const;

function readExplicitRankingArray(
  state: TacticalStatePayload | null | undefined,
): unknown[] | null {
  if (!state) return null;
  const raw = state as Record<string, unknown>;
  for (const key of RANKING_ARRAY_KEYS) {
    const value = raw[key];
    if (Array.isArray(value) && value.length > 0) return value;
  }
  return null;
}

function primaryTargetId(
  state: TacticalStatePayload | null | undefined,
  recommendation: TacticalRecommendationPayload | null | undefined,
): string | null {
  return (
    state?.assigned_target_id ??
    state?.selected_target_id ??
    recommendation?.recommended_target_id ??
    null
  );
}

function recommendationTargetId(
  recommendation: TacticalRecommendationPayload | null | undefined,
  state: TacticalStatePayload | null | undefined,
): string | null {
  return (
    recommendation?.recommended_target_id ??
    state?.assigned_target_id ??
    state?.selected_target_id ??
    null
  );
}

function labelForRankedEntry(rank: number, ttiS: number | null): string {
  const base = rankLabelFor(rank);
  if (rank === 1 && ttiS !== null && Number.isFinite(ttiS)) {
    return `${base} · ${ttiS.toFixed(0)}s`;
  }
  return base;
}

function labelForRecommendationCue(ttiS: number | null): string {
  if (ttiS !== null && Number.isFinite(ttiS)) {
    return `rec #1 · ${ttiS.toFixed(0)}s · cue only`;
  }
  return "rec #1 · recommendation cue only";
}

export function resolveTacticalRankingCues(
  state: TacticalStatePayload | null | undefined,
  recommendation: TacticalRecommendationPayload | null | undefined,
): TacticalRankingCueResolution {
  const ttiS =
    recommendation?.tti_s ??
    state?.tti_s ??
    null;
  const normalizedTti =
    ttiS !== null && ttiS !== undefined && Number.isFinite(Number(ttiS))
      ? Number(ttiS)
      : null;

  const explicitRanking = readExplicitRankingArray(state);
  if (explicitRanking) {
    const targetId = primaryTargetId(state, recommendation);
    const entries = parseTacticalTargetRanking(state, targetId);
    return {
      mode: "ranked_list",
      cues: entries.map((entry) => ({
        ...entry,
        label: labelForRankedEntry(entry.rank, normalizedTti),
      })),
    };
  }

  const recTarget = recommendationTargetId(recommendation, state);
  if (!recTarget) {
    return { mode: "recommendation_only", cues: [] };
  }

  return {
    mode: "recommendation_only",
    cues: [
      {
        entityId: recTarget,
        rank: 1,
        label: labelForRecommendationCue(normalizedTti),
      },
    ],
  };
}

export function formatTacticalRankingSummary(
  resolution: TacticalRankingCueResolution,
): string | null {
  if (resolution.cues.length === 0) return null;
  if (resolution.mode === "ranked_list") {
    return `Ranked cues · ${resolution.cues.length} target(s) · display only`;
  }
  const cue = resolution.cues[0];
  return `Recommendation cue · ${cue.label} · not full ranking`;
}

function removeTacticalRankingEntities(viewer: Viewer): void {
  const toRemove: Entity[] = [];
  viewer.entities.values.forEach((e) => {
    if ((e.id ?? "").startsWith(TACTICAL_RANKING_PREFIX)) toRemove.push(e);
  });
  for (const e of toRemove) viewer.entities.remove(e);
}

function poseFromEntity(
  entity: MirrorEntity | undefined,
): { x: number; y: number; z: number } | null {
  if (!entity) return null;
  const x = Number(entity.pose.x);
  const y = Number(entity.pose.y);
  const z = Number(entity.pose.z);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
    return null;
  }
  return { x, y, z };
}

function rankCueLabelStyle(
  rank: number,
  mode: TacticalRankingCueMode,
  dimmed: boolean,
  alphaScale: number,
) {
  const emphasis = rank === 1;
  const alpha = (dimmed ? 0.52 : emphasis ? 0.98 : 0.82) * alphaScale;
  return {
    font: emphasis ? "11px sans-serif" : "10px sans-serif",
    fillColor: Color.fromCssColorString(
      mode === "recommendation_only"
        ? "rgba(167, 243, 208, 0.96)"
        : rank === 1
          ? "rgba(199, 210, 254, 0.96)"
          : rank === 2
            ? "rgba(165, 180, 252, 0.9)"
            : "rgba(148, 163, 184, 0.85)",
    ).withAlpha(alpha),
    outlineColor: Color.BLACK,
    outlineWidth: emphasis && !dimmed ? 2 : 1,
    style: LabelStyle.FILL_AND_OUTLINE,
    verticalOrigin: VerticalOrigin.CENTER,
    showBackground: true,
    backgroundColor: Color.fromCssColorString(
      mode === "recommendation_only"
        ? "rgba(6, 78, 59, 0.86)"
        : "rgba(30, 27, 75, 0.84)",
    ).withAlpha((dimmed ? 0.65 : 0.88) * alphaScale),
    pixelOffset: new Cartesian2(0, -34 - rank * 5),
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
  };
}

export interface TacticalRankingCueSyncOptions {
  enabled: boolean;
  tacticalState: TacticalStatePayload | null | undefined;
  tacticalRecommendation?: TacticalRecommendationPayload | null | undefined;
  entities: MirrorEntity[];
  selectedEntityId: string | null;
  applyTerrainDisplay: boolean;
  stale?: boolean;
}

export function syncTacticalRankingCueLayer(
  viewer: Viewer | null | undefined,
  options: TacticalRankingCueSyncOptions,
): void {
  if (!isViewerUsable(viewer)) return;
  removeTacticalRankingEntities(viewer);
  if (!options.enabled) return;

  const resolution = resolveTacticalRankingCues(
    options.tacticalState,
    options.tacticalRecommendation,
  );
  if (resolution.cues.length === 0) return;

  const alphaScale = options.stale ? 0.45 : 1;
  const editSelectionActive = Boolean(options.selectedEntityId);

  for (const entry of resolution.cues) {
    if (options.selectedEntityId && entry.entityId === options.selectedEntityId) {
      continue;
    }

    const entity = options.entities.find((e) => e.entity_id === entry.entityId);
    const pose = poseFromEntity(entity);
    if (!pose) continue;

    const dimmed =
      editSelectionActive &&
      options.selectedEntityId !== entry.entityId;
    const displayZ = options.applyTerrainDisplay
      ? applyTerrainDisplayOffset(pose.x, pose.y, pose.z)
      : pose.z;

    viewer.entities.add(
      new Entity({
        id: `${TACTICAL_RANKING_PREFIX}${entry.entityId}`,
        position: worldToCartesian(pose.x, pose.y, displayZ),
        label: {
          text: entry.label,
          ...rankCueLabelStyle(entry.rank, resolution.mode, dimmed, alphaScale),
        },
      }),
    );
  }
}

export function clearTacticalRankingCueLayer(
  viewer: Viewer | null | undefined,
): void {
  if (!isViewerUsable(viewer)) return;
  removeTacticalRankingEntities(viewer);
}
