import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { SensorContributionPanel } from "./SensorContributionPanel";
import { TrackAdvisoryLinkPanel } from "./TrackAdvisoryLinkPanel";
import { TrackConfidencePanel } from "./TrackConfidencePanel";
import { TrackLifecyclePanel } from "./TrackLifecyclePanel";
import { TrackSensorWorkbench } from "./TrackSensorWorkbench";
import type { TrackSensorWorkbenchModel } from "./trackSensorWorkbenchTypes";

function model(options: { stale?: boolean; noAdvisoryLink?: boolean } = {}): TrackSensorWorkbenchModel {
  const stale = options.stale === true;
  return {
    track: {
      track_id: "track-17",
      linked_entity_id: "attacker-a",
      track_state: stale ? "coasting" : "confirmed",
      pose: { x: 1200, y: -420, z: 180 },
      velocity: { vx: -12.5, vy: 3.2, vz: -1.1 },
      heading_deg: 284.5,
      speed_mps: 12.95,
      source_authority: "tracker_mirror_explanatory",
      last_update_utc: stale ? "2026-06-05T10:00:12Z" : "2026-06-05T10:00:30Z",
      staleness: stale ? "stale" : "fresh",
    },
    sensor_contributions: [
      {
        source: "radar",
        status: stale ? "stale" : "present",
        freshness: stale ? "stale" : "fresh",
        contribution: "0.62",
        agreement: "within_gate",
        notes: stale ? "Radar detection aged beyond freshness window." : "Radar contributed to latest fused detection.",
      },
      {
        source: "camera",
        status: "present",
        freshness: "fresh",
        contribution: "0.38",
        agreement: "within_gate",
        notes: "Camera detection agreed with radar within fusion threshold.",
      },
      {
        source: "fused_detection",
        status: "present",
        freshness: stale ? "stale" : "fresh",
        contribution: "paired",
        agreement: "radar_camera_agree",
        notes: "Fused source used paired sensor inputs.",
      },
      {
        source: "tracker_update",
        status: stale ? "coasted" : "updated",
        freshness: stale ? "stale" : "fresh",
        contribution: stale ? "prediction_only" : "measurement_update",
        agreement: stale ? "no_recent_measurement" : "associated",
        notes: stale ? "Track preserved for review while awaiting a fresh detection." : "Detection associated to track-17.",
      },
    ],
    lifecycle_events: [
      {
        event: "first_seen",
        timestamp_utc: "2026-06-05T09:59:40Z",
        reason: "Initial fused detection created track candidate.",
        source: "tracker",
      },
      {
        event: "confirmed",
        timestamp_utc: "2026-06-05T09:59:44Z",
        reason: "Candidate met confirmation threshold.",
        source: "tracker",
      },
      {
        event: "updated",
        timestamp_utc: "2026-06-05T10:00:30Z",
        reason: "Latest fused detection associated with track.",
        source: "tracker",
      },
      {
        event: "coasted",
        timestamp_utc: stale ? "2026-06-05T10:00:34Z" : null,
        reason: stale ? "No fresh detection in latest update window." : "No coast event recorded.",
        source: stale ? "tracker" : "not_available",
      },
      {
        event: "missed",
        timestamp_utc: stale ? "2026-06-05T10:00:35Z" : null,
        reason: stale ? "One expected update was missed." : "No missed event recorded.",
        source: stale ? "tracker" : "not_available",
      },
    ],
    confidence: {
      score: stale ? 0.42 : 0.81,
      level: stale ? "low" : "high",
      factors: stale
        ? ["stale_update", "prediction_only", "sensor_gap"]
        : ["recent_update", "sensor_agreement", "bounded_covariance"],
      basis: stale
        ? "Track quality confidence is reduced because the latest state is stale and prediction-only."
        : "Track quality confidence is high because recent radar and camera inputs agree with bounded covariance.",
    },
    advisory_link: options.noAdvisoryLink
      ? null
      : {
          track_id: "track-17",
          attacker_id: "attacker-a",
          threat_rank: 1,
          threat_score: 48.2,
          recommended_defender: "defender-b",
          freshness_alignment: stale ? "track_stale_advisory_preserved" : "track_and_advisory_fresh",
        },
  };
}

describe("track sensor workbench", () => {
  it("renders active track details without command controls", () => {
    const markup = renderToStaticMarkup(<TrackSensorWorkbench model={model()} />);
    expect(markup).toContain('data-testid="track-sensor-workbench"');
    expect(markup).toContain("Track &amp; sensor workbench");
    expect(markup).toContain("track-17");
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("confirmed");
    expect(markup).toContain("tracker_mirror_explanatory");
    expect(markup).toContain("284.5 deg");
    expect(markup).toContain("12.95 m/s");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
  });

  it("renders stale track state and preserved explanation data", () => {
    const markup = renderToStaticMarkup(<TrackSensorWorkbench model={model({ stale: true })} />);
    expect(markup).toContain("coasting");
    expect(markup).toContain("stale");
    expect(markup).toContain("Track preserved for review");
    expect(markup).toContain("track_stale_advisory_preserved");
  });

  it("renders no advisory link state", () => {
    const markup = renderToStaticMarkup(
      <TrackAdvisoryLinkPanel link={model({ noAdvisoryLink: true }).advisory_link} />,
    );
    expect(markup).toContain("Track");
    expect(markup).toContain("Threat Evaluation");
    expect(markup).toContain("Advisory");
    expect(markup).toContain("No advisory link available for this track.");
  });

  it("renders bounded confidence caveats", () => {
    const markup = renderToStaticMarkup(<TrackConfidencePanel confidence={model().confidence} />);
    expect(markup).toContain("0.81");
    expect(markup).toContain("high");
    expect(markup).toContain("Recent update");
    expect(markup).toContain("Sensor agreement");
    expect(markup).toContain("Not mission success confidence");
    expect(markup).toContain("Not kill probability");
    expect(markup).toContain("Not engagement confidence");
  });

  it("renders lifecycle defaults and optional events", () => {
    const markup = renderToStaticMarkup(
      <TrackLifecyclePanel events={model({ stale: true }).lifecycle_events} />,
    );
    expect(markup).toContain("First seen");
    expect(markup).toContain("Confirmed");
    expect(markup).toContain("Updated");
    expect(markup).toContain("Coasted");
    expect(markup).toContain("Dropped");
    expect(markup).toContain("Missed");
    expect(markup).toContain("No lifecycle event recorded.");
  });

  it("renders sensor contribution rows in required order", () => {
    const markup = renderToStaticMarkup(
      <SensorContributionPanel rows={model().sensor_contributions} />,
    );
    const radar = markup.indexOf("Radar");
    const camera = markup.indexOf("Camera");
    const fused = markup.indexOf("Fused detection");
    const tracker = markup.indexOf("Tracker update");
    expect(radar).toBeGreaterThan(-1);
    expect(camera).toBeGreaterThan(radar);
    expect(fused).toBeGreaterThan(camera);
    expect(tracker).toBeGreaterThan(fused);
    expect(markup).toContain("within_gate");
    expect(markup).toContain("Sensor contribution is explanatory input visibility only");
  });
});
