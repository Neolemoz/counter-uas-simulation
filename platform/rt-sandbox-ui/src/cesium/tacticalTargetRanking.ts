import type { TacticalStatePayload } from "@/bridge/tacticalCommands";

export interface TacticalTargetRankEntry {
  entityId: string;
  rank: number;
}

const MAX_RANK_LABELS = 3;

function parseRankedEntityId(item: unknown): { entityId: string; rank: number } | null {
  if (typeof item === "string" && item.length > 0) {
    return { entityId: item, rank: 0 };
  }
  if (!item || typeof item !== "object") return null;
  const row = item as Record<string, unknown>;
  const entityId =
    (typeof row.entity_id === "string" && row.entity_id) ||
    (typeof row.target_id === "string" && row.target_id) ||
    (typeof row.id === "string" && row.id) ||
    null;
  if (!entityId) return null;
  const rankRaw = row.rank ?? row.ranking ?? row.order;
  const rank = Number(rankRaw);
  return {
    entityId,
    rank: Number.isFinite(rank) && rank > 0 ? Math.floor(rank) : 0,
  };
}

function normalizeRankingList(
  items: unknown[],
  selectedTargetId: string | null,
): TacticalTargetRankEntry[] {
  const entries: TacticalTargetRankEntry[] = [];
  const seen = new Set<string>();

  items.forEach((item, index) => {
    const parsed = parseRankedEntityId(item);
    if (!parsed || seen.has(parsed.entityId)) return;
    seen.add(parsed.entityId);
    entries.push({
      entityId: parsed.entityId,
      rank: parsed.rank > 0 ? parsed.rank : index + 1,
    });
  });

  if (entries.length === 0 && selectedTargetId) {
    return [{ entityId: selectedTargetId, rank: 1 }];
  }

  const hasExplicitRank = entries.some((e) => e.rank > 0);
  if (!hasExplicitRank) {
    entries.forEach((e, i) => {
      e.rank = i + 1;
    });
  }

  if (
    selectedTargetId &&
    !entries.some((e) => e.entityId === selectedTargetId)
  ) {
    entries.unshift({ entityId: selectedTargetId, rank: 1 });
  }

  entries.sort((a, b) => a.rank - b.rank);
  return entries.slice(0, MAX_RANK_LABELS);
}

function readRankingArray(state: Record<string, unknown>): unknown[] | null {
  for (const key of [
    "target_ranking",
    "target_candidates",
    "ranked_targets",
    "candidate_ranking",
    "threat_ranking",
  ]) {
    const raw = state[key];
    if (Array.isArray(raw) && raw.length > 0) return raw;
  }
  return null;
}

export function parseTacticalTargetRanking(
  state: TacticalStatePayload | null | undefined,
  selectedTargetId: string | null,
): TacticalTargetRankEntry[] {
  if (!state) {
    return selectedTargetId ? [{ entityId: selectedTargetId, rank: 1 }] : [];
  }

  const rawState = state as Record<string, unknown>;
  const ranking = readRankingArray(rawState);
  if (ranking) {
    return normalizeRankingList(ranking, selectedTargetId);
  }

  return selectedTargetId ? [{ entityId: selectedTargetId, rank: 1 }] : [];
}

export function rankLabelFor(rank: number): string {
  return `#${rank}`;
}
