import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type { RtIntelligenceAdvisoryTransportV1, RtIntelligenceAdvisoryV1 } from "./intelligenceAdvisory";
import { SelectedTargetAdvisoryCard } from "./SelectedTargetAdvisoryCard";

function advisory(options: { noSolution?: boolean } = {}): RtIntelligenceAdvisoryV1 {
  const noSolution = options.noSolution === true;
  return {
    schema: "rt_intelligence_advisory_v1",
    identity: {
      advisory_id: "adv-attacker-a",
      attacker_id: "attacker-a",
      advisory_utc: "2026-06-05T10:00:00Z",
    },
    threat_evaluation: {
      threat_score: noSolution ? 26.4 : 48.2,
      threat_rank: 1,
      threat_components: {
        distance_to_protected_center: { value_m: 800, normalized: 0.84, weight: 30 },
        best_feasible_tti: { value_s: noSolution ? null : 40, normalized: noSolution ? 0 : 0.6666666667, weight: 30 },
        descent_factor: { value_mps: noSolution ? null : -1, normalized: noSolution ? null : 0.2, weight: 15 },
        critical_zone_factor: { active: false, normalized: 0, weight: 25 },
      },
    },
    recommended_defender: {
      defender_id: noSolution ? null : "defender-b",
      feasibility: {
        feasible: !noSolution,
        reason: noSolution ? "no_solution" : "feasible",
      },
      tti_s: noSolution ? null : 40,
    },
    defender_ranking: {
      ranked_defenders: noSolution
        ? [
            {
              defender_id: "defender-a",
              rank: 1,
              feasible: false,
              tti_s: null,
              reason_codes: [],
            },
          ]
        : [
            {
              defender_id: "defender-b",
              rank: 1,
              feasible: true,
              tti_s: 40,
              reason_codes: ["shortest_tti"],
            },
            {
              defender_id: "defender-a",
              rank: 2,
              feasible: true,
              tti_s: 45,
              reason_codes: [],
            },
          ],
    },
    reasoning: {
      reason_codes: noSolution
        ? ["inside_warning_ring", "no_solution"]
        : ["inside_warning_ring", "descending_fast", "feasible_pair_available", "shortest_tti"],
      explanation: noSolution
        ? "No feasible defender solution is available for attacker attacker-a."
        : "Attacker attacker-a is descending fast; defender defender-b has the shortest feasible TTI.",
    },
    confidence: {
      heuristic_confidence: {
        score: noSolution ? 0.35 : 0.85,
        level: noSolution ? "low" : "high",
        basis: noSolution
          ? ["complete_attacker_identity", "distance_available", "defender_candidates_available"]
          : [
              "complete_attacker_identity",
              "distance_available",
              "descent_available",
              "defender_candidates_available",
              "feasible_tti_available",
              "deterministic_defender_ranking",
            ],
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

function renderSelected(
  payload: RtIntelligenceAdvisoryTransportV1,
  selectedAttackerId = "attacker-a",
): string {
  return renderToStaticMarkup(
    <SelectedTargetAdvisoryCard
      transport={payload}
      selectedAttackerId={selectedAttackerId}
    />,
  );
}

describe("selected target advisory summary integration", () => {
  it("renders active selected advisory summary without standalone workbench ownership", () => {
    const markup = renderSelected(transport([advisory()]));
    expect(markup).toContain('data-testid="selected-target-advisory-card"');
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("defender-b");
    expect(markup).toContain("40.0 s");
    expect(markup).not.toContain('data-testid="threat-evaluation-workbench"');
  });

  it("renders stale selected advisory summary", () => {
    const markup = renderSelected(transport([advisory()], true));
    expect(markup).toContain("Advisory stale");
    expect(markup).toContain("source_stale");
    expect(markup).toContain("attacker-a");
    expect(markup).not.toContain('data-testid="threat-evaluation-workbench"');
  });

  it("renders selected no-solution advisory summary", () => {
    const markup = renderSelected(transport([advisory({ noSolution: true })]));
    expect(markup).toContain("No feasible defender recommendation available.");
    expect(markup).toContain("No feasible defender solution is available");
    expect(markup).toContain("Low heuristic confidence");
    expect(markup).not.toContain('data-testid="threat-evaluation-workbench"');
  });

  it("renders no workbench when selected entity has no advisory", () => {
    const markup = renderSelected(transport([advisory()]), "attacker-missing");
    expect(markup).toContain("No intelligence advisory for selected attacker attacker-missing.");
    expect(markup).not.toContain('data-testid="threat-evaluation-workbench"');
  });

  it("keeps selected summary action-free", () => {
    const markup = renderSelected(transport([advisory()]));
    expect(markup).toContain("Read-only advisory");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/Approve recommendation/);
    expect(markup).not.toMatch(/Assign/);
  });
});
