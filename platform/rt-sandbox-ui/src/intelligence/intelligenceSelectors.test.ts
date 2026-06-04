import { describe, expect, it } from "vitest";
import { advisoryUiState, type RtIntelligenceAdvisoryV1, type RtIntelligenceAdvisoryTransportV1 } from "./intelligenceAdvisory";
import {
  getAdvisoryConfidenceLabel,
  getAdvisoryReasonLabels,
  getRankedAdvisories,
  getSelectedEntityAdvisory,
  getTopAdvisory,
} from "./intelligenceSelectors";

function advisory(
  attackerId: string,
  rank: number | null,
  score: number | null,
  options: {
    defenderId?: string | null;
    confidence?: "low" | "medium" | "high";
    reasonCodes?: string[];
  } = {},
): RtIntelligenceAdvisoryV1 {
  return {
    schema: "rt_intelligence_advisory_v1",
    identity: {
      advisory_id: `adv-${attackerId}`,
      attacker_id: attackerId,
      advisory_utc: "2026-06-05T10:00:00Z",
    },
    threat_evaluation: {
      threat_score: score,
      threat_rank: rank,
      threat_components: {
        distance_to_protected_center: {
          value_m: 1000,
          normalized: 0.8,
          weight: 30,
        },
        best_feasible_tti: {
          value_s: 20,
          normalized: 0.8,
          weight: 30,
        },
        descent_factor: {
          value_mps: 0,
          normalized: 0,
          weight: 15,
        },
        critical_zone_factor: {
          active: false,
          normalized: 0,
          weight: 25,
        },
      },
    },
    recommended_defender: {
      defender_id: options.defenderId ?? "defender-a",
      feasibility: {
        feasible: options.defenderId !== null,
        reason: options.defenderId === null ? "no_solution" : "feasible",
      },
      tti_s: options.defenderId === null ? null : 20,
    },
    defender_ranking: {
      ranked_defenders: [],
    },
    reasoning: {
      reason_codes: options.reasonCodes ?? ["inside_warning_ring", "shortest_tti"],
      explanation: "Display-only advisory.",
    },
    confidence: {
      heuristic_confidence: {
        score: options.confidence === "low" ? 0.35 : options.confidence === "medium" ? 0.6 : 0.85,
        level: options.confidence ?? "high",
        basis: ["test_basis"],
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
    session_id: "session-a",
    advisory_utc: "2026-06-05T10:00:00Z",
    source: "rt_intelligence_advisory_engine",
    authority: "recommendation_only",
    governance_banner:
      "INTELLIGENCE ADVISORY - recommendation only; no assignment, engagement, or weapon authority",
    refresh_reason: "snapshot",
    stale,
    stale_reason: stale ? "source_stale" : null,
    advisories,
  };
}

describe("intelligence advisory selectors", () => {
  it("selects top advisory by threat rank", () => {
    const payload = transport([
      advisory("attacker-b", 2, 90),
      advisory("attacker-a", 1, 70),
    ]);
    expect(getTopAdvisory(payload)?.identity.attacker_id).toBe("attacker-a");
  });

  it("sorts null ranks last and breaks ranked ties by score then attacker id", () => {
    const payload = transport([
      advisory("attacker-z", null, 100),
      advisory("attacker-b", 1, 60),
      advisory("attacker-a", 1, 80),
    ]);
    expect(getRankedAdvisories(payload).map((a) => a.identity.attacker_id)).toEqual([
      "attacker-a",
      "attacker-b",
      "attacker-z",
    ]);
  });

  it("returns no active advisories when transport is stale", () => {
    const payload = transport([advisory("attacker-a", 1, 80)], true);
    expect(getTopAdvisory(payload)).toBeNull();
    expect(getRankedAdvisories(payload)).toEqual([]);
    expect(getSelectedEntityAdvisory(payload, "attacker-a")).toBeNull();
    expect(advisoryUiState(payload)).toBe("stale");
  });

  it("handles empty and loading states", () => {
    expect(advisoryUiState(undefined)).toBe("loading");
    expect(advisoryUiState(transport([]))).toBe("empty");
    expect(getTopAdvisory(transport([]))).toBeNull();
  });

  it("looks up selected attacker by attacker id only", () => {
    const payload = transport([
      advisory("attacker-a", 1, 80, { defenderId: "defender-a" }),
    ]);
    expect(getSelectedEntityAdvisory(payload, "attacker-a")?.identity.attacker_id).toBe(
      "attacker-a",
    );
    expect(getSelectedEntityAdvisory(payload, "defender-a")).toBeNull();
  });

  it("formats confidence labels without operational-success language", () => {
    expect(getAdvisoryConfidenceLabel(advisory("a", 1, 80, { confidence: "high" }))).toBe(
      "High heuristic confidence",
    );
    expect(getAdvisoryConfidenceLabel(advisory("a", 1, 80, { confidence: "medium" }))).toBe(
      "Medium heuristic confidence",
    );
    expect(getAdvisoryConfidenceLabel(advisory("a", 1, 80, { confidence: "low" }))).toBe(
      "Low heuristic confidence",
    );
    expect(getAdvisoryConfidenceLabel(null)).toBe("Confidence unavailable");
  });

  it("maps known and unknown reason codes deterministically", () => {
    const labels = getAdvisoryReasonLabels(
      advisory("a", 1, 80, {
        reasonCodes: [
          "critical_target",
          "shortest_tti",
          "only_feasible",
          "descending_fast",
          "inside_warning_ring",
          "no_solution",
          "feasible_pair_available",
          "tti_tie_break",
          "insufficient_inputs",
          "custom_reason_code",
        ],
      }),
    );
    expect(labels).toEqual([
      "Critical zone",
      "Shortest TTI",
      "Only feasible defender",
      "Descending fast",
      "Inside warning ring",
      "No feasible defender",
      "Feasible pair available",
      "TTI tie break",
      "Insufficient inputs",
      "Custom Reason Code",
    ]);
  });
});
