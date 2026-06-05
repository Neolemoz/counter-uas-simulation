import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { SelectedTrackSensorWorkbench } from "./SelectedTrackSensorWorkbench";
import {
  ACTIVE_TRACK_FIXTURE,
  NO_ADVISORY_TRACK_FIXTURE,
  STALE_TRACK_FIXTURE,
} from "./trackSensorWorkbenchFixtures";
import {
  getSelectedTrackSensorWorkbenchModel,
  isSelectedTrackStale,
  listTrackSensorWorkbenchFixtures,
} from "./trackSensorWorkbenchSelectors";

describe("track sensor workbench fixture binding", () => {
  it("selects active, stale, and no-advisory fixtures without telemetry", () => {
    expect(listTrackSensorWorkbenchFixtures()).toHaveLength(3);
    expect(getSelectedTrackSensorWorkbenchModel("track-17")).toBe(ACTIVE_TRACK_FIXTURE);
    expect(getSelectedTrackSensorWorkbenchModel("track-23")).toBe(STALE_TRACK_FIXTURE);
    expect(getSelectedTrackSensorWorkbenchModel("track-31")).toBe(NO_ADVISORY_TRACK_FIXTURE);
    expect(getSelectedTrackSensorWorkbenchModel("missing-track")).toBeNull();
    expect(isSelectedTrackStale("track-23")).toBe(true);
    expect(isSelectedTrackStale("track-17")).toBe(false);
  });

  it("renders active selected track with advisory linkage", () => {
    const markup = renderToStaticMarkup(
      <SelectedTrackSensorWorkbench selectedTrackId="track-17" />,
    );
    expect(markup).toContain('data-testid="track-sensor-workbench"');
    expect(markup).toContain("track-17");
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("defender-b");
    expect(markup).toContain("track_and_advisory_fresh");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
  });

  it("renders stale selected track while preserving explanation sections", () => {
    const markup = renderToStaticMarkup(
      <SelectedTrackSensorWorkbench selectedTrackId="track-23" />,
    );
    expect(markup).toContain("track-23");
    expect(markup).toContain("coasting");
    expect(markup).toContain("track_stale_advisory_preserved");
    expect(markup).toContain("Track preserved for review");
    expect(markup).toContain("prediction-only");
    expect(markup).toContain("Not mission success confidence");
    expect(markup).toContain("Not kill probability");
    expect(markup).toContain("Not engagement confidence");
  });

  it("switches selected track rendering", () => {
    const activeMarkup = renderToStaticMarkup(
      <SelectedTrackSensorWorkbench selectedTrackId="track-17" />,
    );
    const staleMarkup = renderToStaticMarkup(
      <SelectedTrackSensorWorkbench selectedTrackId="track-23" />,
    );
    expect(activeMarkup).toContain("track-17");
    expect(activeMarkup).not.toContain("track-23");
    expect(staleMarkup).toContain("track-23");
    expect(staleMarkup).not.toContain("track-17");
  });

  it("renders no-advisory selected track", () => {
    const markup = renderToStaticMarkup(
      <SelectedTrackSensorWorkbench selectedTrackId="track-31" />,
    );
    expect(markup).toContain("track-31");
    expect(markup).toContain("attacker-c");
    expect(markup).toContain("No advisory link available for this track.");
    expect(markup).not.toContain("defender-b");
  });

  it("renders empty selected-track state without action controls", () => {
    const markup = renderToStaticMarkup(<SelectedTrackSensorWorkbench selectedTrackId={null} />);
    expect(markup).toContain('data-testid="selected-track-sensor-empty"');
    expect(markup).toContain("Select a track to inspect sensor and track details.");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
  });
});
