import { describe, expect, it } from "vitest";
import type {
  RtIntelligenceAdvisoryTransportV1,
  RtIntelligenceAdvisoryV1,
} from "@/intelligence/intelligenceAdvisory";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { buildLiveTrackSensorWorkbenchModel } from "./liveTrackSensorAdapter";
import { deriveLiveTrackFreshness } from "./liveTrackFreshness";
import { getSelectedLiveTrackSensorWorkbenchModel } from "./liveTrackSensorWorkbenchSelectors";
import {
  TRACK_LIVE_ADAPTER_AUTHORITY,
  TRACK_LIVE_DEGRADED_CONFIDENCE_BASIS,
  TRACK_LIVE_ENTITY_POSE_MIRROR_AUTHORITY,
  TRACK_LIVE_GOVERNANCE_LINES,
  TRACK_LIVE_INTELLIGENCE_ADVISORY_AUTHORITY,
  TRACK_LIVE_LIFECYCLE_UNAVAILABLE_NOTE,
  TRACK_LIVE_SENSOR_UNAVAILABLE_NOTE,
} from "./trackLiveProvenance";

function advisory(attackerId: string): RtIntelligenceAdvisoryV1 {
  return {
    schema: "rt_intelligence_advisory_v1",
    identity: {
      advisory_id: `adv-${attackerId}`,
      attacker_id: attackerId,
      advisory_utc: "2026-06-05T10:00:00Z",
    },
    threat_evaluation: {
      threat_rank: 1,
      threat_score: 63.5,
      threat_components: {
        distance_to_protected_center: { value_m: 850, normalized: 0.7, weight: 30 },
        best_feasible_tti: { value_s: 22, normalized: 0.8, weight: 30 },
        descent_factor: { value_mps: -0.6, normalized: 0.3, weight: 15 },
        critical_zone_factor: { active: false, normalized: 0, weight: 25 },
      },
    },
    recommended_defender: {
      defender_id: "defender-a",
      feasibility: { feasible: true, reason: "feasible" },
      tti_s: 22,
    },
    defender_ranking: {
      ranked_defenders: [
        {
          defender_id: "defender-a",
          rank: 1,
          feasible: true,
          tti_s: 22,
          reason_codes: ["shortest_tti"],
        },
      ],
    },
    reasoning: {
      reason_codes: ["inside_warning_ring", "shortest_tti"],
      explanation: "Display-only advisory.",
    },
    confidence: {
      heuristic_confidence: {
        score: 0.76,
        level: "high",
        basis: ["complete_attacker_identity", "distance_available"],
      },
    },
    governance: {
      authority: "intelligence_advisory_explanatory",
      governance_banner:
        "INTELLIGENCE ADVISORY - recommendation only; no assignment, engagement, or weapon authority",
    },
  };
}

function transport(
  advisories: RtIntelligenceAdvisoryV1[],
  stale = false,
): RtIntelligenceAdvisoryTransportV1 {
  return {
    schema: "rt_intelligence_advisory_transport_v1",
    session_id: "session-live",
    advisory_utc: "2026-06-05T10:00:00Z",
    source: "rt_intelligence_advisory_engine",
    authority: "recommendation_only",
    governance_banner:
      "INTELLIGENCE ADVISORY - recommendation only; no assignment, engagement, or weapon authority",
    refresh_reason: "live_pull",
    stale,
    stale_reason: stale ? "source_stale" : null,
    advisories,
  };
}

function mirror(entityId = "attacker-live"): ChannelSnapshot<"entity_pose_mirror"> {
  return {
    channel: "entity_pose_mirror",
    timestamp_utc: "2026-06-05T10:00:05Z",
    payload: {
      entities: [
        {
          entity_id: entityId,
          entity_type: "drone",
          pose: { x: 100, y: 200, z: 50, yaw_deg: 25 },
          speed_mps: 9.5,
        },
      ],
    },
  };
}

const liveEntity = {
  entity_id: "attacker-live",
  entity_type: "drone",
  pose: { x: 100, y: 200, z: 50, yaw_deg: 25 },
  speed_mps: 9.5,
};

describe("live track sensor adapter", () => {
  it("builds a live linked entity mirror workbench model", () => {
    const model = getSelectedLiveTrackSensorWorkbenchModel({
      selectedEntityId: "attacker-live",
      entityPoseMirror: mirror(),
      intelligenceAdvisory: transport([advisory("attacker-live")]),
      mirrorFreshness: "fresh",
    });

    expect(model?.track).toMatchObject({
      track_id: "attacker-live",
      linked_entity_id: "attacker-live",
      track_state: "live_entity_mirror",
      pose: { x: 100, y: 200, z: 50 },
      heading_deg: 25,
      speed_mps: 9.5,
      source_authority: "entity_pose_mirror_explanatory",
      last_update_utc: "2026-06-05T10:00:05Z",
      track_age_s: null,
      staleness: "fresh",
    });
    expect(model?.advisory_link).toMatchObject({
      track_id: "attacker-live",
      attacker_id: "attacker-live",
      threat_rank: 1,
      threat_score: 63.5,
      recommended_defender: "defender-a",
      freshness_alignment: "fresh",
    });
  });

  it("preserves missing advisory without tactical recommendation substitution", () => {
    const model = getSelectedLiveTrackSensorWorkbenchModel({
      selectedEntityId: "attacker-live",
      entityPoseMirror: mirror(),
      intelligenceAdvisory: transport([]),
      mirrorFreshness: "fresh",
    });

    expect(model?.advisory_link).toBeNull();
    expect(JSON.stringify(model)).not.toContain("tactical_recommendation");
    expect(JSON.stringify(model)).not.toContain("recommendation_id");
    expect(JSON.stringify(model)).not.toContain("recommended_interceptor_id");
  });

  it("marks advisory stale in advisory link freshness without making entity stale", () => {
    const model = getSelectedLiveTrackSensorWorkbenchModel({
      selectedEntityId: "attacker-live",
      entityPoseMirror: mirror(),
      intelligenceAdvisory: transport([advisory("attacker-live")], true),
      mirrorFreshness: "fresh",
    });

    expect(model?.track.staleness).toBe("fresh");
    expect(model?.advisory_link?.freshness_alignment).toBe("advisory_stale");
  });

  it("marks stale mirror as stale entity mirror without tracker lifecycle claims", () => {
    const model = getSelectedLiveTrackSensorWorkbenchModel({
      selectedEntityId: "attacker-live",
      entityPoseMirror: mirror(),
      intelligenceAdvisory: transport([advisory("attacker-live")]),
      mirrorFreshness: "stale",
    });

    expect(model?.track.staleness).toBe("stale");
    expect(model?.track.track_state).toBe("live_entity_mirror");
    expect(model?.lifecycle_events).toEqual([
      {
        event: "stale",
        timestamp_utc: null,
        reason: TRACK_LIVE_LIFECYCLE_UNAVAILABLE_NOTE,
        source: TRACK_LIVE_ADAPTER_AUTHORITY,
      },
    ]);
    expect(model?.lifecycle_events).not.toContainEqual(
      expect.objectContaining({ event: "coasted" }),
    );
  });

  it("keeps confidence unavailable with no synthetic confidence", () => {
    const model = buildLiveTrackSensorWorkbenchModel({
      selectedEntityId: "attacker-live",
      entity: liveEntity,
      advisory: advisory("attacker-live"),
      mirrorSnapshot: mirror(),
      mirrorFreshness: "fresh",
    });

    expect(model?.confidence).toMatchObject({
      level: "unknown",
      score: null,
      basis: TRACK_LIVE_DEGRADED_CONFIDENCE_BASIS,
    });
    expect(model?.confidence.factors).toEqual([
      "no_tracker_confidence_telemetry",
      "no_sensor_fusion_telemetry",
      "no_tracker_lifecycle_telemetry",
    ]);
  });

  it("keeps sensor contribution unavailable without synthetic fusion claims", () => {
    const model = buildLiveTrackSensorWorkbenchModel({
      selectedEntityId: "attacker-live",
      entity: liveEntity,
      advisory: advisory("attacker-live"),
      mirrorSnapshot: mirror(),
      mirrorFreshness: "fresh",
    });

    expect(model?.sensor_contributions).toHaveLength(4);
    expect(model?.sensor_contributions.map((row) => row.status)).toEqual([
      "not_available",
      "not_available",
      "not_available",
      "not_available",
    ]);
    expect(model?.sensor_contributions.every((row) => row.notes === TRACK_LIVE_SENSOR_UNAVAILABLE_NOTE)).toBe(
      true,
    );
    expect(JSON.stringify(model?.sensor_contributions)).not.toContain("paired");
    expect(JSON.stringify(model?.sensor_contributions)).not.toContain("measurement_update");
  });

  it("exports approved provenance constants and governance copy", () => {
    expect(TRACK_LIVE_ENTITY_POSE_MIRROR_AUTHORITY).toBe(
      "entity_pose_mirror_explanatory",
    );
    expect(TRACK_LIVE_INTELLIGENCE_ADVISORY_AUTHORITY).toBe(
      "intelligence_advisory_explanatory",
    );
    expect(TRACK_LIVE_ADAPTER_AUTHORITY).toBe("track_live_adapter_explanatory");
    expect(TRACK_LIVE_GOVERNANCE_LINES).toContain("Live entity mirror workbench");
    expect(TRACK_LIVE_GOVERNANCE_LINES).toContain("Not tracker output");
    expect(TRACK_LIVE_GOVERNANCE_LINES).toContain("No live track telemetry");
    expect(TRACK_LIVE_GOVERNANCE_LINES).toContain("Read-only explanation surface");
    expect(TRACK_LIVE_GOVERNANCE_LINES).toContain("No assignment authority");
    expect(TRACK_LIVE_GOVERNANCE_LINES).toContain("No engagement authority");
    expect(TRACK_LIVE_GOVERNANCE_LINES).toContain("No autonomy authority");
  });

  it("maps live track freshness states from mirror and advisory stale inputs only", () => {
    expect(
      deriveLiveTrackFreshness({
        mirrorFreshness: "fresh",
        advisoryStale: false,
      }).freshness,
    ).toBe("fresh");
    expect(
      deriveLiveTrackFreshness({
        mirrorFreshness: "stale",
        advisoryStale: false,
      }).freshness,
    ).toBe("entity_stale");
    expect(
      deriveLiveTrackFreshness({
        mirrorFreshness: "fresh",
        advisoryStale: true,
        advisoryStaleReason: "source_stale",
      }),
    ).toMatchObject({
      freshness: "advisory_stale",
      advisoryStaleReason: "source_stale",
    });
    expect(
      deriveLiveTrackFreshness({
        mirrorFreshness: "unavailable",
        advisoryStale: true,
      }).freshness,
    ).toBe("both_stale");
    expect(
      deriveLiveTrackFreshness({
        mirrorFreshness: "unknown",
        advisoryStale: false,
      }).freshness,
    ).toBe("unknown");
  });
});
