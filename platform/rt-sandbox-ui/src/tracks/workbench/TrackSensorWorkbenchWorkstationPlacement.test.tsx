import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { TrackSensorWorkbenchWorkstationSurface } from "@/workstation/AppWorkstationSlots";

describe("track sensor workstation placement", () => {
  it("renders standalone workbench surface", () => {
    const markup = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-17" />,
    );
    expect(markup).toContain('data-testid="track-sensor-workstation-surface"');
    expect(markup).toContain('data-testid="track-sensor-workbench"');
    expect(markup).toContain("Track &amp; sensor workbench");
    expect(markup).toContain("TRACK &amp; SENSOR WORKBENCH");
  });

  it("renders empty state without placeholder data", () => {
    const markup = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId={null} />,
    );
    expect(markup).toContain('data-testid="selected-track-sensor-empty"');
    expect(markup).toContain("Select a track to inspect sensor and track details.");
    expect(markup).not.toContain("track-17");
    expect(markup).not.toContain("attacker-a");
  });

  it("renders active selected track", () => {
    const markup = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-17" />,
    );
    expect(markup).toContain("track-17");
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("confirmed");
    expect(markup).toContain("track_and_advisory_fresh");
  });

  it("keeps stale selected track accessible with all explanation sections", () => {
    const markup = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-23" />,
    );
    expect(markup).toContain('data-testid="track-stale-banner"');
    expect(markup).toContain("Track stale - explanation data preserved for review.");
    expect(markup).toContain('data-testid="sensor-contribution-panel"');
    expect(markup).toContain('data-testid="track-lifecycle-panel"');
    expect(markup).toContain('data-testid="track-confidence-panel"');
    expect(markup).toContain("track_stale_advisory_preserved");
  });

  it("switches selected track rendering", () => {
    const activeMarkup = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-17" />,
    );
    const staleMarkup = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-23" />,
    );
    expect(activeMarkup).toContain("track-17");
    expect(activeMarkup).not.toContain("track-23");
    expect(staleMarkup).toContain("track-23");
    expect(staleMarkup).not.toContain("track-17");
  });

  it("does not expose action controls", () => {
    const markup = renderToStaticMarkup(
      <TrackSensorWorkbenchWorkstationSurface selectedTrackId="track-17" />,
    );
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
    expect(markup).toContain("read-only explanation only");
    expect(markup).toContain("no assignment, engagement, or autonomy authority");
  });
});
