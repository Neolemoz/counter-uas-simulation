import { describe, expect, it } from "vitest";
import {
  indexLatestByChannel,
  mergeChannelSnapshots,
  type ChannelSnapshot,
} from "@/telemetry/channelIndex";
import { TELEMETRY_CHANNELS } from "@/telemetry/constants";
import type {
  RtIntelligenceAdvisoryTransportV1,
  RtIntelligenceAdvisoryV1,
} from "./intelligenceAdvisory";
import { advisoryUiState } from "./intelligenceAdvisory";
import {
  getAdvisoryTransportFromSnapshot,
  getRankedAdvisories,
  getSelectedEntityAdvisory,
  getTopAdvisory,
} from "./intelligenceSelectors";

function advisory(attackerId: string, rank: number): RtIntelligenceAdvisoryV1 {
  return {
    schema: "rt_intelligence_advisory_v1",
    identity: {
      advisory_id: `adv-${attackerId}`,
      attacker_id: attackerId,
      advisory_utc: "2026-06-05T10:00:00Z",
    },
    threat_evaluation: {
      threat_score: rank === 1 ? 80 : 60,
      threat_rank: rank,
      threat_components: {
        distance_to_protected_center: { value_m: 800, normalized: 0.84, weight: 30 },
        best_feasible_tti: { value_s: 20, normalized: 0.8, weight: 30 },
        descent_factor: { value_mps: 0, normalized: 0, weight: 15 },
        critical_zone_factor: { active: false, normalized: 0, weight: 25 },
      },
    },
    recommended_defender: {
      defender_id: "defender-a",
      feasibility: { feasible: true, reason: "feasible" },
      tti_s: 20,
    },
    defender_ranking: { ranked_defenders: [] },
    reasoning: {
      reason_codes: ["inside_warning_ring", "shortest_tti"],
      explanation: "Display-only advisory.",
    },
    confidence: {
      heuristic_confidence: {
        score: 0.85,
        level: "high",
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

function snapshot(
  payload: RtIntelligenceAdvisoryTransportV1,
): ChannelSnapshot<"intelligence_advisory"> {
  return {
    channel: "intelligence_advisory",
    timestamp_utc: payload.advisory_utc,
    payload,
  };
}

describe("intelligence advisory telemetry", () => {
  it("subscribes to intelligence advisory with active telemetry channels", () => {
    expect(TELEMETRY_CHANNELS).toContain("intelligence_advisory");
    expect(TELEMETRY_CHANNELS).toContain("tactical_state");
    expect(TELEMETRY_CHANNELS).toContain("tactical_recommendation");
  });

  it("indexes live telemetry snapshot payloads", () => {
    const payload = transport([advisory("attacker-a", 1)]);
    const indexed = indexLatestByChannel([
      {
        channel: "intelligence_advisory",
        session_id: "session-a",
        timestamp_utc: payload.advisory_utc,
        payload,
      },
    ]);
    const parsed = getAdvisoryTransportFromSnapshot(indexed.intelligence_advisory);
    expect(parsed?.schema).toBe("rt_intelligence_advisory_transport_v1");
    expect(getTopAdvisory(parsed)?.identity.attacker_id).toBe("attacker-a");
  });

  it("merges intelligence advisory snapshots into workstation snapshot state", () => {
    const payload = transport([advisory("attacker-a", 1)]);
    const merged = mergeChannelSnapshots({}, [
      {
        channel: "intelligence_advisory",
        session_id: "session-a",
        timestamp_utc: payload.advisory_utc,
        payload,
      },
    ]);
    expect(merged.intelligence_advisory?.payload.session_id).toBe("session-a");
    expect(getSelectedEntityAdvisory(merged.intelligence_advisory?.payload, "attacker-a")).not.toBeNull();
  });

  it("handles empty advisory transport from snapshot", () => {
    const parsed = getAdvisoryTransportFromSnapshot(snapshot(transport([])));
    expect(advisoryUiState(parsed)).toBe("empty");
    expect(getRankedAdvisories(parsed)).toEqual([]);
    expect(getTopAdvisory(parsed)).toBeNull();
  });

  it("handles stale advisory transport from snapshot", () => {
    const parsed = getAdvisoryTransportFromSnapshot(
      snapshot(transport([advisory("attacker-a", 1)], true)),
    );
    expect(advisoryUiState(parsed)).toBe("stale");
    expect(getRankedAdvisories(parsed)).toEqual([]);
    expect(getTopAdvisory(parsed)).toBeNull();
  });

  it("handles populated advisory transport from snapshot", () => {
    const parsed = getAdvisoryTransportFromSnapshot(
      snapshot(transport([advisory("attacker-b", 2), advisory("attacker-a", 1)])),
    );
    expect(advisoryUiState(parsed)).toBe("active");
    expect(getRankedAdvisories(parsed).map((item) => item.identity.attacker_id)).toEqual([
      "attacker-a",
      "attacker-b",
    ]);
  });
});
