import type { RtIntelligenceAdvisoryV1 } from "@/intelligence/intelligenceAdvisory";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { deriveLiveTrackFreshness, type LiveTrackMirrorFreshness } from "./liveTrackFreshness";
import {
  TRACK_LIVE_ADAPTER_AUTHORITY,
  TRACK_LIVE_DEGRADED_CONFIDENCE_BASIS,
  TRACK_LIVE_ENTITY_POSE_MIRROR_AUTHORITY,
  TRACK_LIVE_LIFECYCLE_UNAVAILABLE_NOTE,
  TRACK_LIVE_SENSOR_UNAVAILABLE_NOTE,
} from "./trackLiveProvenance";
import type {
  SensorContributionRow,
  TrackLifecycleEvent,
  TrackSensorWorkbenchModel,
} from "./trackSensorWorkbenchTypes";

type LiveEntityRow = Record<string, unknown>;

export type LiveTrackSensorAdapterInput = {
  selectedEntityId: string | null | undefined;
  entity: LiveEntityRow;
  advisory: RtIntelligenceAdvisoryV1 | null;
  mirrorSnapshot: ChannelSnapshot<"entity_pose_mirror"> | null | undefined;
  mirrorFreshness: LiveTrackMirrorFreshness | null | undefined;
  advisoryStale?: boolean | null;
  advisoryStaleReason?: string | null;
};

function finiteNumber(value: unknown): number | null {
  if (typeof value === "number" && Number.isFinite(value)) return value;
  if (typeof value === "string" && value.trim().length > 0) {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : null;
  }
  return null;
}

function objectOrNull(value: unknown): Record<string, unknown> | null {
  return value && typeof value === "object" ? (value as Record<string, unknown>) : null;
}

function entityId(entity: LiveEntityRow): string | null {
  const id = entity.entity_id;
  if (typeof id !== "string" || id.trim().length === 0) return null;
  return id;
}

function poseValue(entity: LiveEntityRow, key: "x" | "y" | "z"): number | null {
  const position = objectOrNull(entity.position);
  const pose = objectOrNull(entity.pose);
  return finiteNumber(position?.[key]) ?? finiteNumber(pose?.[key]);
}

function headingDeg(entity: LiveEntityRow): number | null {
  const pose = objectOrNull(entity.pose);
  return finiteNumber(entity.heading_deg) ?? finiteNumber(pose?.yaw_deg);
}

function speedMps(entity: LiveEntityRow): number | null {
  const velocity = objectOrNull(entity.velocity);
  return finiteNumber(entity.speed_mps) ?? finiteNumber(velocity?.speed_mps);
}

function unavailableSensorRows(
  freshness: "fresh" | "stale" | "unknown",
): SensorContributionRow[] {
  return ["radar", "camera", "fused_detection", "tracker_update"].map(
    (source): SensorContributionRow => ({
      source: source as SensorContributionRow["source"],
      status: "not_available",
      freshness,
      contribution: "-",
      agreement: "-",
      notes: TRACK_LIVE_SENSOR_UNAVAILABLE_NOTE,
    }),
  );
}

function unavailableLifecycleRows(): TrackLifecycleEvent[] {
  return [
    {
      event: "stale",
      timestamp_utc: null,
      reason: TRACK_LIVE_LIFECYCLE_UNAVAILABLE_NOTE,
      source: TRACK_LIVE_ADAPTER_AUTHORITY,
    },
  ];
}

function buildAdvisoryLink(
  selectedEntityId: string,
  advisory: RtIntelligenceAdvisoryV1 | null,
  freshnessAlignment: string,
): TrackSensorWorkbenchModel["advisory_link"] {
  if (!advisory) return null;
  return {
    track_id: selectedEntityId,
    attacker_id: advisory.identity.attacker_id,
    threat_rank: advisory.threat_evaluation.threat_rank,
    threat_score: advisory.threat_evaluation.threat_score,
    recommended_defender: advisory.recommended_defender.defender_id,
    freshness_alignment: freshnessAlignment,
  };
}

export function buildLiveTrackSensorWorkbenchModel({
  selectedEntityId,
  entity,
  advisory,
  mirrorSnapshot,
  mirrorFreshness,
  advisoryStale = false,
  advisoryStaleReason = null,
}: LiveTrackSensorAdapterInput): TrackSensorWorkbenchModel | null {
  if (!selectedEntityId || selectedEntityId.trim().length === 0) return null;
  if (entityId(entity) !== selectedEntityId) return null;

  const freshness = deriveLiveTrackFreshness({
    mirrorFreshness,
    advisoryStale,
    advisoryStaleReason,
  });

  return {
    track: {
      track_id: selectedEntityId,
      linked_entity_id: selectedEntityId,
      track_state: "live_entity_mirror",
      pose: {
        x: poseValue(entity, "x"),
        y: poseValue(entity, "y"),
        z: poseValue(entity, "z"),
      },
      velocity: {
        vx: null,
        vy: null,
        vz: null,
      },
      heading_deg: headingDeg(entity),
      speed_mps: speedMps(entity),
      source_authority: TRACK_LIVE_ENTITY_POSE_MIRROR_AUTHORITY,
      last_update_utc: mirrorSnapshot?.timestamp_utc ?? null,
      track_age_s: null,
      staleness: freshness.trackStaleness,
    },
    sensor_contributions: unavailableSensorRows(freshness.trackStaleness),
    lifecycle_events: unavailableLifecycleRows(),
    confidence: {
      score: null,
      level: "unknown",
      factors: [
        "no_tracker_confidence_telemetry",
        "no_sensor_fusion_telemetry",
        "no_tracker_lifecycle_telemetry",
      ],
      basis: TRACK_LIVE_DEGRADED_CONFIDENCE_BASIS,
    },
    advisory_link: buildAdvisoryLink(selectedEntityId, advisory, freshness.freshness),
  };
}
