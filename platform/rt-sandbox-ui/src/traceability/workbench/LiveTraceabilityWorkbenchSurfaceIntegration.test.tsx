import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type {
  RtIntelligenceAdvisoryTransportV1,
  RtIntelligenceAdvisoryV1,
} from "@/intelligence/intelligenceAdvisory";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { TrackTraceabilityWorkstationSurface } from "@/workstation/AppWorkstationSlots";
import { LiveTraceabilityWorkbenchSurface } from "./LiveTraceabilityWorkbenchSurface";

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
      threat_score: 64.5,
      threat_components: {
        distance_to_protected_center: { value_m: 800, normalized: 0.8, weight: 30 },
        best_feasible_tti: { value_s: 18, normalized: 0.9, weight: 30 },
        descent_factor: { value_mps: -0.8, normalized: 0.3, weight: 15 },
        critical_zone_factor: { active: false, normalized: 0, weight: 25 },
      },
    },
    recommended_defender: {
      defender_id: "defender-a",
      feasibility: { feasible: true, reason: "feasible" },
      tti_s: 18,
    },
    defender_ranking: {
      ranked_defenders: [
        {
          defender_id: "defender-a",
          rank: 1,
          feasible: true,
          tti_s: 18,
          reason_codes: ["shortest_tti"],
        },
      ],
    },
    reasoning: {
      reason_codes: ["inside_warning_ring", "shortest_tti"],
      explanation: "Live advisory recommendation origin only.",
    },
    confidence: {
      heuristic_confidence: {
        score: 0.78,
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

describe("live traceability workbench surface integration", () => {
  it("renders live linked entity before fixture fallback", () => {
    const markup = renderToStaticMarkup(
      <TrackTraceabilityWorkstationSurface
        selectedTrackId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([advisory("attacker-live")])}
      />,
    );

    expect(markup).toContain('data-testid="live-traceability-provenance"');
    expect(markup).toContain("Live entity/advisory correlation");
    expect(markup).toContain("Not tracker lineage");
    expect(markup).toContain("Fresh");
    expect(markup).toContain("Entity mirror fresh; advisory fresh.");
    expect(markup).toContain("attacker-live");
    expect(markup).toContain("adv-attacker-live");
    expect(markup).toContain("defender-a");
    expect(markup).not.toContain('data-testid="selected-track-traceability-empty"');
  });

  it("renders live missing advisory without synthesizing advisory origin", () => {
    const markup = renderToStaticMarkup(
      <LiveTraceabilityWorkbenchSurface
        selectedEntityId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([])}
        mirrorFreshness="fresh"
      />,
    );

    expect(markup).toContain('data-testid="live-traceability-missing-advisory"');
    expect(markup).toContain("No live advisory available for the selected entity.");
    expect(markup).toContain("Advisory unavailable");
    expect(markup).toContain('data-testid="threat-lineage-missing"');
    expect(markup).toContain('data-testid="advisory-origin-missing"');
    expect(markup).not.toContain("tactical_recommendation");
    expect(markup).not.toContain("recommendation_id");
    expect(markup).not.toContain("recommended_interceptor_id");
  });

  it("renders live stale advisory freshness and stale reason", () => {
    const markup = renderToStaticMarkup(
      <LiveTraceabilityWorkbenchSurface
        selectedEntityId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([advisory("attacker-live")], true)}
        mirrorFreshness="fresh"
      />,
    );

    expect(markup).toContain("Advisory stale");
    expect(markup).toContain("Entity mirror fresh; advisory is marked stale.");
    expect(markup).toContain("source_stale");
    expect(markup).toContain('data-testid="advisory-origin-stale-banner"');
  });

  it("renders live stale mirror freshness through existing traceability UI", () => {
    const markup = renderToStaticMarkup(
      <LiveTraceabilityWorkbenchSurface
        selectedEntityId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([advisory("attacker-live")])}
        mirrorFreshness="stale"
      />,
    );

    expect(markup).toContain("Entity stale");
    expect(markup).toContain("Entity mirror stale; advisory is not marked stale.");
    expect(markup).toContain('data-testid="traceability-stale-banner"');
    expect(markup).toContain("no tracker lifecycle available");
  });

  it("surfaces approved live provenance labels and governance copy", () => {
    const markup = renderToStaticMarkup(
      <LiveTraceabilityWorkbenchSurface
        selectedEntityId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([advisory("attacker-live")])}
        mirrorFreshness="fresh"
      />,
    );

    expect(markup).toContain("entity_pose_mirror_explanatory");
    expect(markup).toContain("intelligence_advisory_explanatory");
    expect(markup).toContain("traceability_live_adapter_explanatory");
    expect(markup).toContain("Live traceability is entity/advisory correlation only");
    expect(markup).toContain("Recommendation origin visibility only");
    expect(markup).toContain("Read-only explanation surface.");
    expect(markup).toContain("No assignment authority");
    expect(markup).toContain("No engagement authority");
    expect(markup).not.toMatch(/<button\b/);
  });

  it("preserves fixture fallback when no live entity model is available", () => {
    const markup = renderToStaticMarkup(
      <TrackTraceabilityWorkstationSurface selectedTrackId="track-17" />,
    );

    expect(markup).toContain('data-testid="selected-track-traceability-workbench"');
    expect(markup).toContain("track-17");
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("Track and advisory fresh");
    expect(markup).not.toContain('data-testid="live-traceability-provenance"');
  });
});
