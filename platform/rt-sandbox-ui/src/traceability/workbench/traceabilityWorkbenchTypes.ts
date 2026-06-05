export type LinkageStatus = "linked" | "partial" | "missing" | "stale" | "mismatch";

export type TrackFreshness = "fresh" | "stale" | "unknown";

export type AdvisoryFreshness = "fresh" | "stale" | "unknown";

export type ConfidenceLevel = "high" | "medium" | "low" | "unknown";

export type TraceabilitySummary = {
  track_id: string;
  attacker_id: string | null;
  threat_rank: number | null;
  threat_score: number | null;
  advisory_id: string | null;
  recommended_defender: string | null;
  linkage_status: LinkageStatus;
  freshness_alignment: string;
};

export type TrackLineage = {
  track_id: string;
  linked_entity_id: string | null;
  attacker_id: string | null;
  track_state: string;
  track_age_s: number | null;
  freshness: TrackFreshness;
  confidence_level: ConfidenceLevel;
  confidence_score: number | null;
  confidence_basis: string;
};

export type ThreatComponentRow = {
  key: string;
  label: string;
  value_display: string;
  normalized: number | null;
  weight: number | null;
};

export type ThreatLineage = {
  attacker_id: string;
  threat_rank: number | null;
  threat_score: number | null;
  threat_components: ThreatComponentRow[];
  heuristic_confidence_level: ConfidenceLevel;
  heuristic_confidence_score: number | null;
  confidence_basis: string[];
};

export type AdvisoryOrigin = {
  advisory_id: string | null;
  attacker_id: string | null;
  recommended_defender: string | null;
  defender_rank: number | null;
  tti_s: number | null;
  reason_codes: string[];
  explanation: string | null;
  advisory_freshness: AdvisoryFreshness;
  advisory_utc: string | null;
  stale_reason: string | null;
};

export type TraceabilityWorkbenchModel = {
  summary: TraceabilitySummary;
  track_lineage: TrackLineage;
  threat_lineage: ThreatLineage | null;
  advisory_origin: AdvisoryOrigin | null;
};
