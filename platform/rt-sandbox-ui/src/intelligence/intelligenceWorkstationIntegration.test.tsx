import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { IntelligenceAdvisoryWorkstationSurfaces } from "@/workstation/AppWorkstationSlots";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type {
  RtIntelligenceAdvisoryTransportV1,
  RtIntelligenceAdvisoryV1,
} from "./intelligenceAdvisory";

function advisory(attackerId: string): RtIntelligenceAdvisoryV1 {
  return {
    schema: "rt_intelligence_advisory_v1",
    identity: {
      advisory_id: `adv-${attackerId}`,
      attacker_id: attackerId,
      advisory_utc: "2026-06-05T10:00:00Z",
    },
    threat_evaluation: {
      threat_score: 80,
      threat_rank: 1,
      threat_components: {
        distance_to_protected_center: { value_m: 800, normalized: 0.84, weight: 30 },
        best_feasible_tti: { value_s: 20, normalized: 0.8, weight: 30 },
        descent_factor: { value_mps: -1, normalized: 0.2, weight: 15 },
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
      explanation: "Defender has the shortest feasible TTI.",
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

function snapshots(payload: RtIntelligenceAdvisoryTransportV1): {
  intelligence_advisory: ChannelSnapshot<"intelligence_advisory">;
} {
  return {
    intelligence_advisory: {
      channel: "intelligence_advisory",
      timestamp_utc: payload.advisory_utc,
      payload,
    },
  };
}

function renderSurfaces(
  payload: RtIntelligenceAdvisoryTransportV1,
  selectedEntityId: string | null = null,
): string {
  return renderToStaticMarkup(
    <IntelligenceAdvisoryWorkstationSurfaces
      snapshots={snapshots(payload)}
      selectedEntityId={selectedEntityId}
    />,
  );
}

describe("workstation intelligence advisory integration", () => {
  it("renders empty advisory transport in the workstation surface", () => {
    const markup = renderSurfaces(transport([]));
    expect(markup).toContain('data-testid="intelligence-advisory-panel"');
    expect(markup).toContain("No current intelligence advisories.");
    expect(markup).not.toMatch(/<button\b/);
  });

  it("renders active advisory transport in the workstation surface", () => {
    const markup = renderSurfaces(transport([advisory("attacker-a")]));
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("Threat #1");
    expect(markup).toContain("defender-a");
    expect(markup).toContain("High heuristic confidence");
    expect(markup).toContain("Read-only advisory");
  });

  it("renders stale advisory transport in the workstation surface", () => {
    const markup = renderSurfaces(transport([advisory("attacker-a")], true));
    expect(markup).toContain("Advisory stale");
    expect(markup).toContain("source_stale");
  });

  it("shows selected attacker advisory when selected entity matches", () => {
    const markup = renderSurfaces(
      transport([advisory("attacker-a")]),
      "attacker-a",
    );
    expect(markup).toContain('data-testid="selected-target-advisory-card"');
    expect(markup).toContain("Defender has the shortest feasible TTI.");
  });

  it("hides selected attacker advisory when selected entity has no match", () => {
    const markup = renderSurfaces(
      transport([advisory("attacker-a")]),
      "attacker-missing",
    );
    expect(markup).toContain('data-testid="intelligence-advisory-panel"');
    expect(markup).not.toContain('data-testid="selected-target-advisory-card"');
    expect(markup).not.toContain("No intelligence advisory for selected attacker");
  });
});
