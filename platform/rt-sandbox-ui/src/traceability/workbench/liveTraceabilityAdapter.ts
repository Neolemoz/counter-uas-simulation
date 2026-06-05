import type { RtIntelligenceAdvisoryV1 } from "@/intelligence/intelligenceAdvisory";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type { TrackSensorWorkbenchModel } from "@/tracks/workbench/trackSensorWorkbenchTypes";
import type { TraceabilityAssemblyInput } from "./traceabilitySelectors";
import {
  deriveLiveTraceabilityFreshness,
  type LiveMirrorFreshness,
} from "./liveTraceabilityFreshness";
import {
  TRACEABILITY_DEGRADED_CONFIDENCE_BASIS,
  TRACEABILITY_ENTITY_POSE_MIRROR_AUTHORITY,
} from "./traceabilityProvenance";

type LiveEntityRow = Record<string, unknown>;

export type LiveTraceabilityAdapterInput = {
  selectedEntityId: string | null | undefined;
  entity: LiveEntityRow;
  advisory: RtIntelligenceAdvisoryV1 | null;
  mirrorSnapshot: ChannelSnapshot<"entity_pose_mirror"> | null | undefined;
  mirrorFreshness: LiveMirrorFreshness | null | undefined;
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

function entityId(entity: LiveEntityRow): string | null {
  const id = entity.entity_id;
  if (typeof id !== "string" || id.trim().length === 0) return null;
  return id;
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

export function buildLiveTraceabilityAssemblyInput({
  selectedEntityId,
  entity,
  advisory,
  mirrorSnapshot,
  mirrorFreshness,
  advisoryStale = false,
  advisoryStaleReason = null,
}: LiveTraceabilityAdapterInput): TraceabilityAssemblyInput | null {
  if (!selectedEntityId || selectedEntityId.trim().length === 0) return null;
  if (entityId(entity) !== selectedEntityId) return null;

  const freshness = deriveLiveTraceabilityFreshness({
    mirrorFreshness,
    advisoryStale,
    advisoryStaleReason,
  });

  return {
    trackModel: {
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
        source_authority: TRACEABILITY_ENTITY_POSE_MIRROR_AUTHORITY,
        last_update_utc: mirrorSnapshot?.timestamp_utc ?? null,
        track_age_s: null,
        staleness: freshness.trackStaleness,
      },
      sensor_contributions: [],
      lifecycle_events: [],
      confidence: {
        score: null,
        level: "unknown",
        factors: [
          "no_tracker_confidence_available",
          "no_sensor_fusion_evidence_available",
          "no_tracker_lifecycle_available",
        ],
        basis: TRACEABILITY_DEGRADED_CONFIDENCE_BASIS,
      },
      advisory_link: buildAdvisoryLink(selectedEntityId, advisory, freshness.freshness),
    },
    advisory,
    metadata: {
      advisory_stale: freshness.advisoryStale,
      advisory_stale_reason: freshness.advisoryStaleReason,
    },
  };
}
