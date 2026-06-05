import type { TrackSensorWorkbenchModel } from "./trackSensorWorkbenchTypes";

export const ACTIVE_TRACK_FIXTURE: TrackSensorWorkbenchModel = {
  track: {
    track_id: "track-17",
    linked_entity_id: "attacker-a",
    track_state: "confirmed",
    pose: { x: 1200, y: -420, z: 180 },
    velocity: { vx: -12.5, vy: 3.2, vz: -1.1 },
    heading_deg: 284.5,
    speed_mps: 12.95,
    source_authority: "tracker_mirror_explanatory",
    last_update_utc: "2026-06-05T10:00:30Z",
    track_age_s: 50,
    staleness: "fresh",
  },
  sensor_contributions: [
    {
      source: "radar",
      status: "present",
      freshness: "fresh",
      contribution: "0.62",
      agreement: "within_gate",
      notes: "Radar contributed to latest fused detection.",
    },
    {
      source: "camera",
      status: "present",
      freshness: "fresh",
      contribution: "0.38",
      agreement: "within_gate",
      notes: "Camera detection agreed with radar within fusion threshold.",
    },
    {
      source: "fused_detection",
      status: "present",
      freshness: "fresh",
      contribution: "paired",
      agreement: "radar_camera_agree",
      notes: "Fused source used paired sensor inputs.",
    },
    {
      source: "tracker_update",
      status: "updated",
      freshness: "fresh",
      contribution: "measurement_update",
      agreement: "associated",
      notes: "Detection associated to track-17.",
    },
  ],
  lifecycle_events: [
    {
      event: "first_seen",
      timestamp_utc: "2026-06-05T09:59:40Z",
      reason: "Initial fused detection created track candidate.",
      source: "tracker",
    },
    {
      event: "confirmed",
      timestamp_utc: "2026-06-05T09:59:44Z",
      reason: "Candidate met confirmation threshold.",
      source: "tracker",
    },
    {
      event: "updated",
      timestamp_utc: "2026-06-05T10:00:30Z",
      reason: "Latest fused detection associated with track.",
      source: "tracker",
    },
  ],
  confidence: {
    score: 0.81,
    level: "high",
    factors: ["recent_update", "sensor_agreement", "bounded_covariance"],
    basis:
      "Track quality confidence is high because recent radar and camera inputs agree with bounded covariance.",
  },
  advisory_link: {
    track_id: "track-17",
    attacker_id: "attacker-a",
    threat_rank: 1,
    threat_score: 48.2,
    recommended_defender: "defender-b",
    freshness_alignment: "track_and_advisory_fresh",
  },
};

export const STALE_TRACK_FIXTURE: TrackSensorWorkbenchModel = {
  track: {
    ...ACTIVE_TRACK_FIXTURE.track,
    track_id: "track-23",
    linked_entity_id: "attacker-b",
    track_state: "coasting",
    last_update_utc: "2026-06-05T10:00:12Z",
    track_age_s: 74,
    staleness: "stale",
  },
  sensor_contributions: [
    {
      source: "radar",
      status: "stale",
      freshness: "stale",
      contribution: "0.62",
      agreement: "within_gate",
      notes: "Radar detection aged beyond freshness window.",
    },
    {
      source: "camera",
      status: "missing",
      freshness: "stale",
      contribution: "-",
      agreement: "sensor_gap",
      notes: "No fresh camera detection is available for the selected track.",
    },
    {
      source: "fused_detection",
      status: "stale",
      freshness: "stale",
      contribution: "last_known_pairing",
      agreement: "no_recent_pair",
      notes: "Last fused detection is preserved for review.",
    },
    {
      source: "tracker_update",
      status: "coasted",
      freshness: "stale",
      contribution: "prediction_only",
      agreement: "no_recent_measurement",
      notes: "Track preserved for review while awaiting a fresh detection.",
    },
  ],
  lifecycle_events: [
    ...ACTIVE_TRACK_FIXTURE.lifecycle_events.map((event) =>
      event.event === "updated"
        ? {
            ...event,
            timestamp_utc: "2026-06-05T10:00:12Z",
            reason: "Last fused detection associated before the track became stale.",
          }
        : event,
    ),
    {
      event: "coasted",
      timestamp_utc: "2026-06-05T10:00:34Z",
      reason: "No fresh detection in latest update window.",
      source: "tracker",
    },
    {
      event: "missed",
      timestamp_utc: "2026-06-05T10:00:35Z",
      reason: "One expected update was missed.",
      source: "tracker",
    },
  ],
  confidence: {
    score: 0.42,
    level: "low",
    factors: ["stale_update", "prediction_only", "sensor_gap"],
    basis:
      "Track quality confidence is reduced because the latest state is stale and prediction-only.",
  },
  advisory_link: {
    track_id: "track-23",
    attacker_id: "attacker-b",
    threat_rank: 2,
    threat_score: 35.7,
    recommended_defender: "defender-c",
    freshness_alignment: "track_stale_advisory_preserved",
  },
};

export const NO_ADVISORY_TRACK_FIXTURE: TrackSensorWorkbenchModel = {
  track: {
    ...ACTIVE_TRACK_FIXTURE.track,
    track_id: "track-31",
    linked_entity_id: "attacker-c",
    heading_deg: 91.2,
    speed_mps: 8.4,
    track_age_s: 38,
  },
  sensor_contributions: ACTIVE_TRACK_FIXTURE.sensor_contributions.map((row) =>
    row.source === "tracker_update"
      ? { ...row, notes: "Detection associated to track-31." }
      : row,
  ),
  lifecycle_events: ACTIVE_TRACK_FIXTURE.lifecycle_events,
  confidence: {
    score: 0.74,
    level: "medium",
    factors: ["recent_update", "single_advisory_gap", "bounded_covariance"],
    basis:
      "Track quality confidence is medium because the track is recent, but no advisory link is available.",
  },
  advisory_link: null,
};

export const TRACK_SENSOR_WORKBENCH_FIXTURES: readonly TrackSensorWorkbenchModel[] = [
  ACTIVE_TRACK_FIXTURE,
  STALE_TRACK_FIXTURE,
  NO_ADVISORY_TRACK_FIXTURE,
];
