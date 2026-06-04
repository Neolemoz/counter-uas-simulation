export type AdvisoryConfidenceLevel = "low" | "medium" | "high";

export type AdvisoryUiState = "empty" | "loading" | "stale" | "active";

export type AdvisoryFeasibility = {
  feasible: boolean;
  reason: string;
};

export type ThreatComponentValue = {
  value_m?: number | null;
  value_s?: number | null;
  value_mps?: number | null;
  active?: boolean;
  normalized: number | null;
  weight: number;
};

export type RtIntelligenceAdvisoryV1 = {
  schema: "rt_intelligence_advisory_v1";
  identity: {
    advisory_id: string;
    attacker_id: string;
    advisory_utc: string;
  };
  threat_evaluation: {
    threat_score: number | null;
    threat_rank: number | null;
    threat_components: {
      distance_to_protected_center: ThreatComponentValue;
      best_feasible_tti: ThreatComponentValue;
      descent_factor: ThreatComponentValue;
      critical_zone_factor: ThreatComponentValue;
    };
  };
  recommended_defender: {
    defender_id: string | null;
    feasibility: AdvisoryFeasibility;
    tti_s: number | null;
  };
  defender_ranking: {
    ranked_defenders: Array<{
      defender_id: string;
      rank: number;
      feasible: boolean;
      tti_s: number | null;
      reason_codes: string[];
    }>;
  };
  reasoning: {
    reason_codes: string[];
    explanation: string;
  };
  confidence: {
    heuristic_confidence: {
      score: number;
      level: AdvisoryConfidenceLevel;
      basis: string[];
    };
  };
  governance: {
    authority: "intelligence_advisory_explanatory";
    governance_banner: string;
  };
};

export type RtIntelligenceAdvisoryTransportV1 = {
  schema: "rt_intelligence_advisory_transport_v1";
  session_id: string;
  advisory_utc: string;
  source: "rt_intelligence_advisory_engine";
  authority: "recommendation_only";
  governance_banner: string;
  refresh_reason: string;
  stale: boolean;
  stale_reason: string | null;
  advisories: RtIntelligenceAdvisoryV1[];
};

export function isIntelligenceAdvisoryTransport(
  value: unknown,
): value is RtIntelligenceAdvisoryTransportV1 {
  if (!value || typeof value !== "object") return false;
  const raw = value as Record<string, unknown>;
  return (
    raw.schema === "rt_intelligence_advisory_transport_v1" &&
    raw.source === "rt_intelligence_advisory_engine" &&
    raw.authority === "recommendation_only" &&
    Array.isArray(raw.advisories)
  );
}

export function advisoryUiState(
  transport: RtIntelligenceAdvisoryTransportV1 | null | undefined,
): AdvisoryUiState {
  if (!transport) return "loading";
  if (transport.stale) return "stale";
  if (transport.advisories.length === 0) return "empty";
  return "active";
}
