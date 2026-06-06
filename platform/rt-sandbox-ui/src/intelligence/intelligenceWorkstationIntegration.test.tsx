import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { IntelligenceAdvisoryWorkstationSurfaces } from "@/workstation/AppWorkstationSlots";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import type {
  RtIntelligenceAdvisoryTransportV1,
  RtIntelligenceAdvisoryV1,
} from "./intelligenceAdvisory";

function advisory(
  attackerId: string,
  options: {
    rank?: number;
    score?: number;
    defenderId?: string | null;
    noSolution?: boolean;
  } = {},
): RtIntelligenceAdvisoryV1 {
  const noSolution = options.noSolution === true || options.defenderId === null;
  const defenderId = noSolution ? null : options.defenderId ?? "defender-a";
  return {
    schema: "rt_intelligence_advisory_v1",
    identity: {
      advisory_id: `adv-${attackerId}`,
      attacker_id: attackerId,
      advisory_utc: "2026-06-05T10:00:00Z",
    },
    threat_evaluation: {
      threat_score: options.score ?? (noSolution ? 26.4 : 80),
      threat_rank: options.rank ?? 1,
      threat_components: {
        distance_to_protected_center: { value_m: 800, normalized: 0.84, weight: 30 },
        best_feasible_tti: { value_s: noSolution ? null : 20, normalized: noSolution ? 0 : 0.8, weight: 30 },
        descent_factor: { value_mps: noSolution ? null : -1, normalized: noSolution ? null : 0.2, weight: 15 },
        critical_zone_factor: { active: false, normalized: 0, weight: 25 },
      },
    },
    recommended_defender: {
      defender_id: defenderId,
      feasibility: { feasible: !noSolution, reason: noSolution ? "no_solution" : "feasible" },
      tti_s: noSolution ? null : 20,
    },
    defender_ranking: {
      ranked_defenders: [
        {
          defender_id: defenderId ?? "defender-unavailable",
          rank: 1,
          feasible: !noSolution,
          tti_s: noSolution ? null : 20,
          reason_codes: noSolution ? [] : ["shortest_tti"],
        },
      ],
    },
    reasoning: {
      reason_codes: noSolution ? ["inside_warning_ring", "no_solution"] : ["inside_warning_ring", "shortest_tti"],
      explanation: noSolution
        ? `No feasible defender solution is available for attacker ${attackerId}.`
        : `Defender ${defenderId} has the shortest feasible TTI.`,
    },
    confidence: {
      heuristic_confidence: {
        score: noSolution ? 0.35 : 0.85,
        level: noSolution ? "low" : "high",
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
  staleReason: string | null = stale ? "source_stale" : null,
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
    stale_reason: staleReason,
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
  protectedCenterEntityId: string | null = null,
): string {
  return renderToStaticMarkup(
    <IntelligenceAdvisoryWorkstationSurfaces
      snapshots={snapshots(payload)}
      selectedEntityId={selectedEntityId}
      protectedCenterEntityId={protectedCenterEntityId}
      entities={
        protectedCenterEntityId
          ? [
              {
                entity_id: protectedCenterEntityId,
                entity_type: "waypoint_marker",
                pose: { x: 0, y: 0, z: 5 },
              },
            ]
          : []
      }
    />,
  );
}

describe("workstation intelligence advisory integration", () => {
  it("renders empty advisory transport with standalone workbench empty state", () => {
    const markup = renderSurfaces(transport([]));
    expect(markup).toContain('data-testid="intelligence-advisory-panel"');
    expect(markup).toContain('data-testid="threat-evaluation-workbench-empty"');
    expect(markup).toContain("No current intelligence advisories.");
    expect(markup).toContain("Select an attacker to inspect threat evaluation details.");
    expect(markup).toContain("INTELLIGENCE ADVISORY");
    expect(markup).not.toMatch(/<button\b/);
  });

  it("renders standalone workbench for active selected advisory", () => {
    const markup = renderSurfaces(
      transport([advisory("attacker-a", { score: 72.5, defenderId: "defender-a" })]),
      "attacker-a",
    );
    expect(markup).toContain('data-testid="selected-target-advisory-card"');
    expect(markup).toContain('data-testid="threat-evaluation-workbench"');
    expect(markup).toContain("Threat evaluation workbench");
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("Threat score");
    expect(markup).toContain("72.5");
    expect(markup).toContain("Defender defender-a has the shortest feasible TTI.");
  });

  it("renders protected center status strip", () => {
    const markup = renderSurfaces(transport([]), null, "center-a");
    expect(markup).toContain('data-testid="protected-center-status-strip"');
    expect(markup).toContain('data-testid="protected-center-designated"');
    expect(markup).toContain("center-a");
    expect(markup).toContain("Waypoint");
  });

  it("renders protected_center_unavailable copy in advisory panel", () => {
    const markup = renderSurfaces(
      transport([], true, "protected_center_unavailable"),
    );
    expect(markup).toContain('data-testid="intelligence-protected-center-unavailable"');
    expect(markup).toContain("restore live threat evaluation");
    expect(markup).not.toContain("Advisory stale - review as historical");
  });

  it("renders stale advisory transport and keeps selected workbench accessible", () => {
    const markup = renderSurfaces(
      transport([advisory("attacker-a", { score: 64.2 })], true),
      "attacker-a",
    );
    expect(markup).toContain("Advisory stale");
    expect(markup).toContain("source_stale");
    expect(markup).toContain('data-testid="threat-evaluation-workbench"');
    expect(markup).toContain("explanation data preserved for review: source_stale");
    expect(markup).toContain("64.2");
  });

  it("renders standalone workbench no-solution selected advisory", () => {
    const markup = renderSurfaces(
      transport([advisory("attacker-a", { noSolution: true })]),
      "attacker-a",
    );
    expect(markup).toContain('data-testid="threat-evaluation-workbench"');
    expect(markup).toContain("No feasible defender recommendation available.");
    expect(markup).toContain("No feasible defender solution is available for attacker attacker-a.");
    expect(markup).toContain("Not kill probability");
  });

  it("renders standalone empty state when selected entity has no advisory", () => {
    const markup = renderSurfaces(
      transport([advisory("attacker-a")]),
      "attacker-missing",
    );
    expect(markup).toContain('data-testid="intelligence-advisory-panel"');
    expect(markup).toContain('data-testid="threat-evaluation-workbench-empty"');
    expect(markup).toContain("Select an attacker to inspect threat evaluation details.");
    expect(markup).not.toContain('data-testid="selected-target-advisory-card"');
  });

  it("switches standalone workbench with selected advisory", () => {
    const payload = transport([
      advisory("attacker-a", { score: 51.1, defenderId: "defender-a", rank: 2 }),
      advisory("attacker-b", { score: 88.8, defenderId: "defender-b", rank: 1 }),
    ]);
    const attackerA = renderSurfaces(payload, "attacker-a");
    const attackerB = renderSurfaces(payload, "attacker-b");

    expect(attackerA).toContain("51.1");
    expect(attackerA).toContain("Defender defender-a has the shortest feasible TTI.");
    expect(attackerB).toContain("88.8");
    expect(attackerB).toContain("Defender defender-b has the shortest feasible TTI.");
  });
});
