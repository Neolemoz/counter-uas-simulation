import type { RtIntelligenceAdvisoryV1 } from "@/intelligence/intelligenceAdvisory";
import {
  ACTIVE_TRACK_FIXTURE,
  NO_ADVISORY_TRACK_FIXTURE,
  STALE_TRACK_FIXTURE,
} from "@/tracks/workbench/trackSensorWorkbenchFixtures";
import type { TrackSensorWorkbenchModel } from "@/tracks/workbench/trackSensorWorkbenchTypes";
import type { TraceabilityAssemblyInput } from "./traceabilitySelectors";

type AdvisoryOptions = {
  rank?: number | null;
  score?: number | null;
  defenderId?: string | null;
  ttiS?: number | null;
  noSolution?: boolean;
  confidenceLevel?: "low" | "medium" | "high";
  confidenceScore?: number;
  confidenceBasis?: string[];
  reasonCodes?: string[];
  explanation?: string;
  advisoryUtc?: string;
  rankedDefenders?: RtIntelligenceAdvisoryV1["defender_ranking"]["ranked_defenders"];
};

export function buildTraceabilityAdvisory(
  attackerId: string,
  options: AdvisoryOptions = {},
): RtIntelligenceAdvisoryV1 {
  const noSolution = options.noSolution === true;
  const defenderId = noSolution ? null : (options.defenderId ?? "defender-b");
  const rank = options.rank ?? 1;
  const score = options.score ?? 48.2;
  const confidenceLevel = options.confidenceLevel ?? (noSolution ? "medium" : "high");
  const confidenceScore = options.confidenceScore ?? (noSolution ? 0.62 : 0.85);

  return {
    schema: "rt_intelligence_advisory_v1",
    identity: {
      advisory_id: `adv-${attackerId}`,
      attacker_id: attackerId,
      advisory_utc: options.advisoryUtc ?? "2026-06-05T10:00:00Z",
    },
    threat_evaluation: {
      threat_score: score,
      threat_rank: rank,
      threat_components: {
        distance_to_protected_center: { value_m: 800, normalized: 0.84, weight: 30 },
        best_feasible_tti: {
          value_s: noSolution ? null : (options.ttiS ?? 40),
          normalized: noSolution ? 0 : 0.67,
          weight: 30,
        },
        descent_factor: {
          value_mps: noSolution ? null : -1,
          normalized: noSolution ? null : 0.2,
          weight: 15,
        },
        critical_zone_factor: { active: false, normalized: 0, weight: 25 },
      },
    },
    recommended_defender: {
      defender_id: defenderId,
      feasibility: {
        feasible: !noSolution,
        reason: noSolution ? "no_solution" : "feasible",
      },
      tti_s: noSolution ? null : (options.ttiS ?? 40),
    },
    defender_ranking: {
      ranked_defenders:
        options.rankedDefenders ??
        (noSolution
          ? []
          : [
              {
                defender_id: defenderId ?? "defender-b",
                rank: 1,
                feasible: true,
                tti_s: options.ttiS ?? 40,
                reason_codes: ["shortest_tti"],
              },
            ]),
    },
    reasoning: {
      reason_codes: options.reasonCodes ?? ["inside_warning_ring", "descending_fast", "shortest_tti"],
      explanation:
        options.explanation ??
        (noSolution
          ? `No feasible defender solution is available for attacker ${attackerId}.`
          : `Attacker ${attackerId} is descending fast; defender ${defenderId} has the shortest feasible TTI.`),
    },
    confidence: {
      heuristic_confidence: {
        score: confidenceScore,
        level: confidenceLevel,
        basis:
          options.confidenceBasis ??
          (noSolution
            ? ["complete_attacker_identity", "distance_available"]
            : [
                "complete_attacker_identity",
                "distance_available",
                "descent_available",
                "defender_candidates_available",
                "feasible_tti_available",
                "deterministic_defender_ranking",
              ]),
      },
    },
    governance: {
      authority: "intelligence_advisory_explanatory",
      governance_banner:
        "INTELLIGENCE ADVISORY - recommendation only; no assignment, engagement, or weapon authority",
    },
  };
}

const MISSING_ADVISORY_TRACK_MODEL: TrackSensorWorkbenchModel = {
  track: {
    track_id: "track-42",
    linked_entity_id: "attacker-d",
    track_state: "confirmed",
    pose: { x: 900, y: -300, z: 160 },
    velocity: { vx: -8.2, vy: 1.4, vz: -0.6 },
    heading_deg: 210.4,
    speed_mps: 8.35,
    source_authority: "tracker_mirror_explanatory",
    last_update_utc: "2026-06-05T10:00:28Z",
    track_age_s: 42,
    staleness: "fresh",
  },
  sensor_contributions: ACTIVE_TRACK_FIXTURE.sensor_contributions,
  lifecycle_events: ACTIVE_TRACK_FIXTURE.lifecycle_events,
  confidence: {
    score: 0.68,
    level: "medium",
    factors: ["recent_update", "advisory_gap", "bounded_covariance"],
    basis: "Track is recent, but no threat evaluation or advisory linkage is available.",
  },
  advisory_link: null,
};

const MISMATCH_TRACK_MODEL: TrackSensorWorkbenchModel = {
  track: {
    ...ACTIVE_TRACK_FIXTURE.track,
    track_id: "track-55",
    linked_entity_id: "attacker-e",
    track_age_s: 45,
    staleness: "fresh",
  },
  sensor_contributions: ACTIVE_TRACK_FIXTURE.sensor_contributions,
  lifecycle_events: ACTIVE_TRACK_FIXTURE.lifecycle_events,
  confidence: {
    score: 0.55,
    level: "medium",
    factors: ["recent_update", "linkage_mismatch", "bounded_covariance"],
    basis:
      "Track links to attacker-e, but advisory origin references a different attacker identity.",
  },
  advisory_link: null,
};

export const TRACEABILITY_FIXTURE_INPUTS = {
  fullyLinked: {
    trackModel: ACTIVE_TRACK_FIXTURE,
    advisory: buildTraceabilityAdvisory("attacker-a", {
      defenderId: "defender-b",
      ttiS: 40,
      explanation:
        "Attacker attacker-a is descending fast; defender defender-b has the shortest feasible TTI.",
      reasonCodes: [
        "inside_warning_ring",
        "descending_fast",
        "feasible_pair_available",
        "shortest_tti",
      ],
    }),
  },
  partial: {
    trackModel: NO_ADVISORY_TRACK_FIXTURE,
    advisory: buildTraceabilityAdvisory("attacker-c", {
      rank: 2,
      score: 35.7,
      noSolution: true,
      confidenceLevel: "medium",
      confidenceScore: 0.62,
      reasonCodes: ["inside_warning_ring", "no_solution"],
    }),
  },
  missingAdvisory: {
    trackModel: MISSING_ADVISORY_TRACK_MODEL,
    advisory: null,
  },
  staleAdvisory: {
    trackModel: STALE_TRACK_FIXTURE,
    advisory: buildTraceabilityAdvisory("attacker-b", {
      rank: 2,
      score: 35.7,
      defenderId: "defender-c",
      ttiS: 52,
      confidenceLevel: "medium",
      confidenceScore: 0.58,
      confidenceBasis: ["complete_attacker_identity", "distance_available"],
      reasonCodes: ["inside_warning_ring", "feasible_pair_available"],
      advisoryUtc: "2026-06-05T09:58:00Z",
      rankedDefenders: [
        {
          defender_id: "defender-c",
          rank: 1,
          feasible: true,
          tti_s: 52,
          reason_codes: ["shortest_tti"],
        },
      ],
    }),
    metadata: {
      advisory_stale: true,
      advisory_stale_reason: "track_stale_beyond_freshness_window",
    },
  },
  mismatch: {
    trackModel: MISMATCH_TRACK_MODEL,
    advisory: buildTraceabilityAdvisory("attacker-f", {
      defenderId: "defender-a",
      ttiS: 45,
      confidenceLevel: "medium",
      confidenceScore: 0.6,
      confidenceBasis: ["complete_attacker_identity", "distance_available"],
      reasonCodes: ["inside_warning_ring"],
      advisoryUtc: "2026-06-05T10:00:05Z",
      rankedDefenders: [
        {
          defender_id: "defender-a",
          rank: 1,
          feasible: true,
          tti_s: 45,
          reason_codes: ["shortest_tti"],
        },
      ],
    }),
  },
} as const satisfies Record<string, TraceabilityAssemblyInput>;
