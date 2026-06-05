import { describe, expect, it } from "vitest";
import type {
  RtIntelligenceAdvisoryTransportV1,
  RtIntelligenceAdvisoryV1,
} from "@/intelligence/intelligenceAdvisory";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { buildLiveTraceabilityAssemblyInput } from "./liveTraceabilityAdapter";
import { deriveLiveTraceabilityFreshness } from "./liveTraceabilityFreshness";
import {
  getLiveTraceabilityAssemblyInput,
  getSelectedLiveTraceabilityModel,
} from "./liveTraceabilitySelectors";
import {
  TRACEABILITY_ENTITY_POSE_MIRROR_AUTHORITY,
  TRACEABILITY_INTELLIGENCE_ADVISORY_AUTHORITY,
  TRACEABILITY_LIVE_ADAPTER_AUTHORITY,
  TRACEABILITY_LIVE_GOVERNANCE_LINES,
} from "./traceabilityProvenance";
import { assembleTraceabilityWorkbenchModel } from "./traceabilitySelectors";

function advisory(
  attackerId: string,
  options: {
    defenderId?: string | null;
    rank?: number | null;
    score?: number | null;
  } = {},
): RtIntelligenceAdvisoryV1 {
  const defenderId = options.defenderId ?? "defender-a";
  return {
    schema: "rt_intelligence_advisory_v1",
    identity: {
      advisory_id: `adv-${attackerId}`,
      attacker_id: attackerId,
      advisory_utc: "2026-06-05T10:00:00Z",
    },
    threat_evaluation: {
      threat_rank: options.rank ?? 1,
      threat_score: options.score ?? 72.5,
      threat_components: {
        distance_to_protected_center: { value_m: 900, normalized: 0.7, weight: 30 },
        best_feasible_tti: { value_s: 21, normalized: 0.8, weight: 30 },
        descent_factor: { value_mps: -1.2, normalized: 0.4, weight: 15 },
        critical_zone_factor: { active: false, normalized: 0, weight: 25 },
      },
    },
    recommended_defender: {
      defender_id: defenderId,
      feasibility: { feasible: defenderId !== null, reason: defenderId ? "feasible" : "no_solution" },
      tti_s: defenderId ? 21 : null,
    },
    defender_ranking: {
      ranked_defenders: defenderId
        ? [
            {
              defender_id: defenderId,
              rank: 1,
              feasible: true,
              tti_s: 21,
              reason_codes: ["shortest_tti"],
            },
          ]
        : [],
    },
    reasoning: {
      reason_codes: ["inside_warning_ring", "shortest_tti"],
      explanation: "Recommendation origin is display-only.",
    },
    confidence: {
      heuristic_confidence: {
        score: 0.82,
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
  options: { stale?: boolean; staleReason?: string | null } = {},
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
    stale: options.stale ?? false,
    stale_reason: options.staleReason ?? null,
    advisories,
  };
}

function mirror(
  entities: Array<Record<string, unknown>>,
  timestampUtc = "2026-06-05T10:00:05Z",
): ChannelSnapshot<"entity_pose_mirror"> {
  return {
    channel: "entity_pose_mirror",
    payload: {
      entities,
      source: "live",
    },
    timestamp_utc: timestampUtc,
  };
}

const liveEntity = {
  entity_id: "attacker-a",
  entity_type: "drone",
  pose: { x: 100, y: 200, z: 30, yaw_deg: 45 },
  speed_mps: 12.4,
};

describe("live traceability adapter", () => {
  it("builds linked live entity/advisory correlation without tracker evidence", () => {
    const model = getSelectedLiveTraceabilityModel({
      selectedEntityId: "attacker-a",
      entityPoseMirror: mirror([liveEntity]),
      intelligenceAdvisory: transport([advisory("attacker-a")]),
      mirrorFreshness: "fresh",
    });

    expect(model?.summary).toMatchObject({
      track_id: "attacker-a",
      attacker_id: "attacker-a",
      advisory_id: "adv-attacker-a",
      recommended_defender: "defender-a",
      linkage_status: "linked",
      freshness_alignment: "fresh",
    });
    expect(model?.track_lineage).toMatchObject({
      track_id: "attacker-a",
      linked_entity_id: "attacker-a",
      track_state: "live_entity_mirror",
      freshness: "fresh",
      confidence_level: "unknown",
      confidence_score: null,
    });
    expect(model?.track_lineage.confidence_basis).toContain(
      "No tracker confidence available",
    );
    expect(model?.track_lineage.confidence_basis).toContain(
      "no sensor fusion evidence available",
    );
    expect(model?.track_lineage.confidence_basis).toContain(
      "no tracker lifecycle available",
    );
  });

  it("preserves missing advisory without synthesizing threat lineage", () => {
    const model = getSelectedLiveTraceabilityModel({
      selectedEntityId: "attacker-a",
      entityPoseMirror: mirror([liveEntity]),
      intelligenceAdvisory: transport([]),
      mirrorFreshness: "fresh",
    });

    expect(model?.summary).toMatchObject({
      track_id: "attacker-a",
      attacker_id: "attacker-a",
      advisory_id: null,
      linkage_status: "missing",
      freshness_alignment: "advisory_unavailable",
    });
    expect(model?.threat_lineage).toBeNull();
    expect(model?.advisory_origin).toBeNull();
  });

  it("preserves stale advisory metadata from the live transport", () => {
    const model = getSelectedLiveTraceabilityModel({
      selectedEntityId: "attacker-a",
      entityPoseMirror: mirror([liveEntity]),
      intelligenceAdvisory: transport([advisory("attacker-a")], {
        stale: true,
        staleReason: "source_stale",
      }),
      mirrorFreshness: "fresh",
    });

    expect(model?.summary).toMatchObject({
      linkage_status: "stale",
      freshness_alignment: "advisory_stale",
    });
    expect(model?.track_lineage.freshness).toBe("fresh");
    expect(model?.advisory_origin).toMatchObject({
      advisory_id: "adv-attacker-a",
      advisory_freshness: "stale",
      stale_reason: "source_stale",
    });
  });

  it("marks live entity mirror stale without inventing tracker lifecycle", () => {
    const input = getLiveTraceabilityAssemblyInput({
      selectedEntityId: "attacker-a",
      entityPoseMirror: mirror([liveEntity]),
      intelligenceAdvisory: transport([advisory("attacker-a")]),
      mirrorFreshness: "stale",
    });
    const model = input ? assembleTraceabilityWorkbenchModel(input) : null;

    expect(input?.trackModel.lifecycle_events).toEqual([]);
    expect(input?.trackModel.sensor_contributions).toEqual([]);
    expect(model?.summary).toMatchObject({
      linkage_status: "stale",
      freshness_alignment: "entity_stale",
    });
    expect(model?.track_lineage.freshness).toBe("stale");
    expect(model?.threat_lineage?.confidence_basis).toContain("stale_track_context");
  });

  it("surfaces live entity/advisory mismatch without rewriting advisory identity", () => {
    const input = buildLiveTraceabilityAssemblyInput({
      selectedEntityId: "attacker-a",
      entity: liveEntity,
      advisory: advisory("attacker-b"),
      mirrorSnapshot: mirror([liveEntity]),
      mirrorFreshness: "fresh",
    });
    const model = input ? assembleTraceabilityWorkbenchModel(input) : null;

    expect(model?.summary).toMatchObject({
      track_id: "attacker-a",
      attacker_id: "attacker-a",
      advisory_id: "adv-attacker-b",
      linkage_status: "mismatch",
      freshness_alignment: "attacker_id_mismatch",
    });
    expect(model?.advisory_origin?.attacker_id).toBe("attacker-b");
    expect(model?.advisory_origin?.reason_codes).toContain("linkage_mismatch");
  });

  it("exports approved provenance labels and governance copy", () => {
    expect(TRACEABILITY_ENTITY_POSE_MIRROR_AUTHORITY).toBe(
      "entity_pose_mirror_explanatory",
    );
    expect(TRACEABILITY_INTELLIGENCE_ADVISORY_AUTHORITY).toBe(
      "intelligence_advisory_explanatory",
    );
    expect(TRACEABILITY_LIVE_ADAPTER_AUTHORITY).toBe(
      "traceability_live_adapter_explanatory",
    );
    expect(TRACEABILITY_LIVE_GOVERNANCE_LINES).toContain(
      "Live traceability is entity/advisory correlation only; it is not tracker lineage.",
    );
    expect(TRACEABILITY_LIVE_GOVERNANCE_LINES).toContain(
      "Recommendation origin visibility only.",
    );
    expect(TRACEABILITY_LIVE_GOVERNANCE_LINES).toContain("No assignment authority.");
    expect(TRACEABILITY_LIVE_GOVERNANCE_LINES).toContain("No engagement authority.");
  });

  it("maps live freshness states from mirror and advisory stale inputs only", () => {
    expect(
      deriveLiveTraceabilityFreshness({
        mirrorFreshness: "fresh",
        advisoryStale: false,
      }).freshness,
    ).toBe("fresh");
    expect(
      deriveLiveTraceabilityFreshness({
        mirrorFreshness: "stale",
        advisoryStale: false,
      }).freshness,
    ).toBe("entity_stale");
    expect(
      deriveLiveTraceabilityFreshness({
        mirrorFreshness: "fresh",
        advisoryStale: true,
        advisoryStaleReason: "source_stale",
      }),
    ).toMatchObject({
      freshness: "advisory_stale",
      advisoryStaleReason: "source_stale",
    });
    expect(
      deriveLiveTraceabilityFreshness({
        mirrorFreshness: "unavailable",
        advisoryStale: true,
      }).freshness,
    ).toBe("both_stale");
    expect(
      deriveLiveTraceabilityFreshness({
        mirrorFreshness: "unknown",
        advisoryStale: false,
      }).freshness,
    ).toBe("unknown");
  });
});
