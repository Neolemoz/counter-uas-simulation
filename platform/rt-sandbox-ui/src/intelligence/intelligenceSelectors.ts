import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import {
  isIntelligenceAdvisoryTransport,
  type RtIntelligenceAdvisoryTransportV1,
  type RtIntelligenceAdvisoryV1,
} from "./intelligenceAdvisory";

const REASON_LABELS: Record<string, string> = {
  critical_target: "Critical zone",
  shortest_tti: "Shortest TTI",
  only_feasible: "Only feasible defender",
  descending_fast: "Descending fast",
  inside_warning_ring: "Inside warning ring",
  no_solution: "No feasible defender",
  feasible_pair_available: "Feasible pair available",
  tti_tie_break: "TTI tie break",
  insufficient_inputs: "Insufficient inputs",
};

function advisorySortKey(advisory: RtIntelligenceAdvisoryV1): {
  rank: number;
  score: number;
  attackerId: string;
} {
  const rank = advisory.threat_evaluation.threat_rank;
  const score = advisory.threat_evaluation.threat_score;
  return {
    rank: typeof rank === "number" && Number.isFinite(rank) ? rank : Number.POSITIVE_INFINITY,
    score: typeof score === "number" && Number.isFinite(score) ? score : Number.NEGATIVE_INFINITY,
    attackerId: advisory.identity.attacker_id,
  };
}

function activeAdvisories(
  transport: RtIntelligenceAdvisoryTransportV1 | null | undefined,
): RtIntelligenceAdvisoryV1[] {
  if (!transport || transport.stale) return [];
  return transport.advisories;
}

export function getAdvisoryTransportFromSnapshot(
  snapshot: ChannelSnapshot | null | undefined,
): RtIntelligenceAdvisoryTransportV1 | null {
  if (!snapshot || snapshot.channel !== "intelligence_advisory") return null;
  return isIntelligenceAdvisoryTransport(snapshot.payload) ? snapshot.payload : null;
}

export function getRankedAdvisories(
  transport: RtIntelligenceAdvisoryTransportV1 | null | undefined,
): RtIntelligenceAdvisoryV1[] {
  return [...activeAdvisories(transport)].sort((a, b) => {
    const ak = advisorySortKey(a);
    const bk = advisorySortKey(b);
    return (
      ak.rank - bk.rank ||
      bk.score - ak.score ||
      ak.attackerId.localeCompare(bk.attackerId)
    );
  });
}

export function getTopAdvisory(
  transport: RtIntelligenceAdvisoryTransportV1 | null | undefined,
): RtIntelligenceAdvisoryV1 | null {
  return getRankedAdvisories(transport)[0] ?? null;
}

export function getSelectedEntityAdvisory(
  transport: RtIntelligenceAdvisoryTransportV1 | null | undefined,
  selectedEntityId: string | null | undefined,
): RtIntelligenceAdvisoryV1 | null {
  if (!selectedEntityId) return null;
  return (
    activeAdvisories(transport).find(
      (advisory) => advisory.identity.attacker_id === selectedEntityId,
    ) ?? null
  );
}

export function getAdvisoryConfidenceLabel(
  advisory: RtIntelligenceAdvisoryV1 | null | undefined,
): string {
  const level = advisory?.confidence.heuristic_confidence.level;
  if (level === "high") return "High heuristic confidence";
  if (level === "medium") return "Medium heuristic confidence";
  if (level === "low") return "Low heuristic confidence";
  return "Confidence unavailable";
}

function humanizeReasonCode(code: string): string {
  return code
    .split("_")
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(" ");
}

export function getAdvisoryReasonLabels(
  advisory: RtIntelligenceAdvisoryV1 | null | undefined,
): string[] {
  const codes = advisory?.reasoning.reason_codes ?? [];
  return codes.map((code) => REASON_LABELS[code] ?? humanizeReasonCode(code));
}
