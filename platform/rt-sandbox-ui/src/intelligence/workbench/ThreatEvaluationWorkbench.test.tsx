import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type { RtIntelligenceAdvisoryV1 } from "../intelligenceAdvisory";
import { ConfidenceExplanationPanel } from "./ConfidenceExplanationPanel";
import { DefenderComparisonPanel } from "./DefenderComparisonPanel";
import { RecommendationExplanationPanel } from "./RecommendationExplanationPanel";
import { ThreatBreakdownPanel } from "./ThreatBreakdownPanel";
import { ThreatEvaluationWorkbench } from "./ThreatEvaluationWorkbench";

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

describe("threat evaluation workbench", () => {
  it("renders the container without command controls", () => {
    const markup = renderToStaticMarkup(<ThreatEvaluationWorkbench advisory={advisory()} />);
    expect(markup).toContain('data-testid="threat-evaluation-workbench"');
    expect(markup).toContain("Threat evaluation workbench");
    expect(markup).toContain("INTELLIGENCE ADVISORY");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
  });

  it("renders threat breakdown rows and derived contributions", () => {
    const markup = renderToStaticMarkup(<ThreatBreakdownPanel advisory={advisory()} />);
    expect(markup).toContain("Distance to protected center");
    expect(markup).toContain("800.0 m");
    expect(markup).toContain("0.84");
    expect(markup).toContain("30.0");
    expect(markup).toContain("25.2");
    expect(markup).toContain("Best feasible TTI");
    expect(markup).toContain("20.0");
    expect(markup).toContain("3.0");
  });

  it("renders defender rows in payload order", () => {
    const markup = renderToStaticMarkup(<DefenderComparisonPanel advisory={advisory()} />);
    const first = markup.indexOf("defender-b");
    const second = markup.indexOf("defender-a");
    expect(first).toBeGreaterThan(-1);
    expect(second).toBeGreaterThan(first);
    expect(markup).toContain("#1");
    expect(markup).toContain("40.0 s");
    expect(markup).toContain("Shortest TTI");
  });

  it("renders recommendation rationale", () => {
    const markup = renderToStaticMarkup(<RecommendationExplanationPanel advisory={advisory()} />);
    expect(markup).toContain("defender-b");
    expect(markup).toContain("40.0 s");
    expect(markup).toContain("descending fast");
    expect(markup).toContain("Inside warning ring");
    expect(markup).toContain("Shortest TTI");
  });

  it("renders confidence score, level, basis, and bounded caveats", () => {
    const markup = renderToStaticMarkup(<ConfidenceExplanationPanel advisory={advisory()} />);
    expect(markup).toContain("0.85");
    expect(markup).toContain("high");
    expect(markup).toContain("Complete attacker identity");
    expect(markup).toContain("Feasible tti available");
    expect(markup).toContain("Not mission success confidence");
    expect(markup).toContain("Not kill probability");
  });

  it("renders no-solution advisory state", () => {
    const markup = renderToStaticMarkup(<ThreatEvaluationWorkbench advisory={advisory({ noSolution: true })} />);
    expect(markup).toContain("No feasible defender recommendation available.");
    expect(markup).toContain("No feasible defender solution is available");
    expect(markup).toContain("no_solution");
    expect(markup).toContain("low");
    expect(markup).toContain("-");
  });
});
