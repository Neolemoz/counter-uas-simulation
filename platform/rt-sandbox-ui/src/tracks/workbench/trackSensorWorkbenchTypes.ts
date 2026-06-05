export type TrackFreshness = "fresh" | "stale" | "unknown";

export type TrackDetail = {
  track_id: string;
  linked_entity_id: string | null;
  track_state: string;
  pose: {
    x: number | null;
    y: number | null;
    z: number | null;
  };
  velocity: {
    vx: number | null;
    vy: number | null;
    vz: number | null;
  };
  heading_deg: number | null;
  speed_mps: number | null;
  source_authority: string;
  last_update_utc: string | null;
  track_age_s: number | null;
  staleness: TrackFreshness;
};

export type SensorContributionSource =
  | "radar"
  | "camera"
  | "fused_detection"
  | "tracker_update";

export type SensorContributionRow = {
  source: SensorContributionSource;
  status: string;
  freshness: TrackFreshness;
  contribution: string;
  agreement: string;
  notes: string;
};

export type TrackLifecycleEventKind =
  | "first_seen"
  | "confirmed"
  | "updated"
  | "coasted"
  | "dropped"
  | "missed"
  | "reacquired"
  | "merged"
  | "stale";

export type TrackLifecycleEvent = {
  event: TrackLifecycleEventKind;
  timestamp_utc: string | null;
  reason: string;
  source: string;
};

export type TrackConfidence = {
  score: number | null;
  level: "high" | "medium" | "low" | "unknown";
  factors: string[];
  basis: string;
};

export type TrackAdvisoryLink = {
  track_id: string;
  attacker_id: string | null;
  threat_rank: number | null;
  threat_score: number | null;
  recommended_defender: string | null;
  freshness_alignment: string;
};

export type TrackSensorWorkbenchModel = {
  track: TrackDetail;
  sensor_contributions: SensorContributionRow[];
  lifecycle_events: TrackLifecycleEvent[];
  confidence: TrackConfidence;
  advisory_link: TrackAdvisoryLink | null;
};
