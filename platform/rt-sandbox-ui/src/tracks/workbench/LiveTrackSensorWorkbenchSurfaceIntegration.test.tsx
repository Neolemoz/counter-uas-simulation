import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type {
  RtIntelligenceAdvisoryTransportV1,
  RtIntelligenceAdvisoryV1,
} from "@/intelligence/intelligenceAdvisory";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { TrackSensorWorkbenchWorkstationSurface } from "@/workstation/AppWorkstationSlots";
import { LiveTrackSensorWorkbenchSurface } from "./LiveTrackSensorWorkbenchSurface";

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

describe("live track sensor workbench surface integration", () => {
  it("renders live linked entity before fixture fallback", () => {
    const markup = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface
        selectedTrackId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([advisory("attacker-live")])}
      />,
    );

    expect(markup).toContain('data-testid="live-track-provenance"');
    expect(markup).toContain("Live entity mirror workbench");
    expect(markup).toContain("Not tracker output");
    expect(markup).toContain("No live track telemetry");
    expect(markup).toContain("Entity/advisory correlation only");
    expect(markup).toContain("attacker-live");
    expect(markup).toContain("defender-a");
    expect(markup).toContain("live_entity_mirror");
    expect(markup).toContain("Entity mirror fresh; advisory fresh.");
  });

  it("renders live missing advisory without tactical recommendation substitution", () => {
    const markup = renderToStaticMarkup(
      <LiveTrackSensorWorkbenchSurface
        selectedEntityId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([])}
        mirrorFreshness="fresh"
      />,
    );

    expect(markup).toContain('data-testid="live-track-missing-advisory"');
    expect(markup).toContain("No linked live advisory available for the selected entity.");
    expect(markup).toContain("No advisory link available for this track.");
    expect(markup).not.toContain("tactical_recommendation");
    expect(markup).not.toContain("recommendation_id");
    expect(markup).not.toContain("recommended_interceptor_id");
  });

  it("renders live stale advisory without marking entity mirror stale", () => {
    const markup = renderToStaticMarkup(
      <LiveTrackSensorWorkbenchSurface
        selectedEntityId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([advisory("attacker-live")], true)}
        mirrorFreshness="fresh"
      />,
    );

    expect(markup).toContain("advisory_stale");
    expect(markup).toContain("Entity mirror fresh; advisory is marked stale.");
    expect(markup).toContain("Fresh");
    expect(markup).not.toContain('data-testid="track-stale-banner"');
  });

  it("renders live stale mirror without synthetic tracker lifecycle", () => {
    const markup = renderToStaticMarkup(
      <LiveTrackSensorWorkbenchSurface
        selectedEntityId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([advisory("attacker-live")])}
        mirrorFreshness="stale"
      />,
    );

    expect(markup).toContain('data-testid="track-stale-banner"');
    expect(markup).toContain("entity_stale");
    expect(markup).toContain("Entity mirror stale; advisory is not marked stale.");
    expect(markup).toContain("Tracker lifecycle unavailable.");
    expect(markup).toContain("No live tracker lifecycle telemetry exists");
    expect(markup).not.toContain("coasted");
    expect(markup).not.toContain("confirmed");
  });

  it("surfaces unavailable confidence, lifecycle, and sensor contribution", () => {
    const markup = renderToStaticMarkup(
      <LiveTrackSensorWorkbenchSurface
        selectedEntityId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([advisory("attacker-live")])}
        mirrorFreshness="fresh"
      />,
    );

    expect(markup).toContain("Track confidence unavailable.");
    expect(markup).toContain("Sensor contribution unavailable.");
    expect(markup).toContain("Tracker lifecycle unavailable.");
    expect(markup).toContain("No tracker confidence telemetry exists");
    expect(markup).toContain("No live sensor contribution telemetry exists");
    expect(markup).toContain("No live tracker lifecycle telemetry exists");
    expect(markup).toContain("unknown");
    expect(markup).not.toContain("sensor_agreement");
    expect(markup).not.toContain("bounded_covariance");
    expect(markup).not.toContain("measurement_update");
    expect(markup).not.toContain("Candidate met confirmation threshold");
  });

  it("surfaces approved provenance labels and governance copy", () => {
    const markup = renderToStaticMarkup(
      <LiveTrackSensorWorkbenchSurface
        selectedEntityId="attacker-live"
        entityPoseMirror={mirror()}
        intelligenceAdvisory={transport([advisory("attacker-live")])}
        mirrorFreshness="fresh"
      />,
    );

    expect(markup).toContain("entity_pose_mirror_explanatory");
    expect(markup).toContain("intelligence_advisory_explanatory");
    expect(markup).toContain("track_live_adapter_explanatory");
    expect(markup).toContain("Read-only explanation surface");
    expect(markup).toContain("Entity/advisory correlation only");
    expect(markup).toContain("No assignment authority");
    expect(markup).toContain("No engagement authority");
    expect(markup).toContain("No autonomy authority");
    expect(markup).not.toMatch(/<button\b/);
  });

  it("preserves fixture fallback for frozen fixture ids", () => {
    const active = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-17" />,
    );
    const stale = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-23" />,
    );
    const noAdvisory = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-31" />,
    );
    const missing42 = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-42" />,
    );
    const missing55 = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-55" />,
    );

    expect(active).toContain("track-17");
    expect(active).toContain("track_and_advisory_fresh");
    expect(active).not.toContain('data-testid="live-track-provenance"');
    expect(stale).toContain("track-23");
    expect(stale).toContain("track_stale_advisory_preserved");
    expect(noAdvisory).toContain("track-31");
    expect(noAdvisory).toContain("No advisory link available for this track.");
    expect(missing42).toContain('data-testid="selected-track-sensor-empty"');
    expect(missing55).toContain('data-testid="selected-track-sensor-empty"');
  });
});
