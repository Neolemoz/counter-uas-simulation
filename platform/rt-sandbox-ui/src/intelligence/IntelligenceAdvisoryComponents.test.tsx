import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type { RtIntelligenceAdvisoryTransportV1, RtIntelligenceAdvisoryV1 } from "./intelligenceAdvisory";
import { IntelligenceAdvisoryPanel } from "./IntelligenceAdvisoryPanel";
import { IntelligenceAdvisoryStrip } from "./IntelligenceAdvisoryStrip";
import { SelectedTargetAdvisoryCard } from "./SelectedTargetAdvisoryCard";

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
        distance_to_protected_center: { value_m: 800, normalized: 0.84, weight: 30 },
        best_feasible_tti: { value_s: 20, normalized: 0.8, weight: 30 },
        descent_factor: { value_mps: -1, normalized: 0.2, weight: 15 },
        critical_zone_factor: { active: false, normalized: 0, weight: 25 },
      },
    },
    recommended_defender: {
      defender_id: Object.hasOwn(options, "defenderId") ? (options.defenderId ?? null) : "defender-a",
      feasibility: {
        feasible: options.defenderId !== null,
        reason: options.defenderId === null ? "no_solution" : "feasible",
      },
      tti_s: options.defenderId === null ? null : 20,
    },
    defender_ranking: { ranked_defenders: [] },
    reasoning: {
      reason_codes: options.reasonCodes ?? ["inside_warning_ring", "shortest_tti"],
      explanation: "Defender has the shortest feasible TTI.",
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

describe("intelligence advisory components", () => {
  it("renders protected_center_unavailable copy without generic stale wording", () => {
    const payload = transport([], true);
    payload.stale_reason = "protected_center_unavailable";
    const panel = renderToStaticMarkup(<IntelligenceAdvisoryPanel transport={payload} />);
    expect(panel).toContain('data-testid="intelligence-protected-center-unavailable"');
    expect(panel).toContain("restore live threat evaluation");
    expect(panel).not.toContain("Advisory stale - review as historical");

    const strip = renderToStaticMarkup(<IntelligenceAdvisoryStrip transport={payload} />);
    expect(strip).toContain(
      'data-testid="intelligence-advisory-strip-protected-center-unavailable"',
    );
    expect(strip).toContain("restore live threat evaluation");
  });

  it("renders panel empty state as read-only", () => {
    const markup = renderToStaticMarkup(
      <IntelligenceAdvisoryPanel transport={transport([])} />,
    );
    expect(markup).toContain('data-testid="intelligence-advisory-panel"');
    expect(markup).toContain("No current intelligence advisories.");
    expect(markup).toContain("Read-only advisory");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
  });

  it("renders stale state and stale reason", () => {
    const payload = transport([advisory("attacker-a", 1, 80)], true);
    const panel = renderToStaticMarkup(<IntelligenceAdvisoryPanel transport={payload} />);
    const strip = renderToStaticMarkup(<IntelligenceAdvisoryStrip transport={payload} />);
    expect(panel).toContain("Advisory stale");
    expect(panel).toContain("source_stale");
    expect(strip).toContain("Advisory stale");
  });

  it("renders active panel rows with confidence and reason labels", () => {
    const payload = transport([
      advisory("attacker-b", 2, 60, { confidence: "medium" }),
      advisory("attacker-a", 1, 80, {
        reasonCodes: ["critical_target", "descending_fast", "shortest_tti"],
      }),
    ]);
    const markup = renderToStaticMarkup(<IntelligenceAdvisoryPanel transport={payload} />);
    expect(markup).toContain("2 advisories");
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("Threat #1");
    expect(markup).toContain("80.0");
    expect(markup).toContain("defender-a");
    expect(markup).toContain("20.0 s");
    expect(markup).toContain("High heuristic confidence");
    expect(markup).toContain("Critical zone");
    expect(markup).toContain("Descending fast");
  });

  it("renders compact strip with top advisory", () => {
    const markup = renderToStaticMarkup(
      <IntelligenceAdvisoryStrip
        transport={transport([
          advisory("attacker-b", 2, 60),
          advisory("attacker-a", 1, 80),
        ])}
      />,
    );
    expect(markup).toContain('data-testid="intelligence-advisory-strip"');
    expect(markup).toContain("#1 attacker-a");
    expect(markup).toContain("defender-a");
    expect(markup).toContain("2 advisories");
  });

  it("renders selected attacker match", () => {
    const markup = renderToStaticMarkup(
      <SelectedTargetAdvisoryCard
        transport={transport([advisory("attacker-a", 1, 80)])}
        selectedAttackerId="attacker-a"
      />,
    );
    expect(markup).toContain('data-testid="selected-target-advisory-card"');
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("defender-a");
    expect(markup).toContain("High heuristic confidence");
    expect(markup).toContain("Inside warning ring");
  });

  it("renders selected attacker missing placeholder", () => {
    const markup = renderToStaticMarkup(
      <SelectedTargetAdvisoryCard
        transport={transport([advisory("attacker-a", 1, 80)])}
        selectedAttackerId="attacker-missing"
      />,
    );
    expect(markup).toContain("No intelligence advisory for selected attacker attacker-missing.");
  });

  it("renders no-solution recommendation text", () => {
    const markup = renderToStaticMarkup(
      <SelectedTargetAdvisoryCard
        transport={transport([
          advisory("attacker-a", 1, 30, {
            defenderId: null,
            confidence: "low",
            reasonCodes: ["no_solution"],
          }),
        ])}
        selectedAttackerId="attacker-a"
      />,
    );
    expect(markup).toContain("No feasible defender recommendation available.");
    expect(markup).toContain("Low heuristic confidence");
    expect(markup).toContain("No feasible defender");
  });
});
