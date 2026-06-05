import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { SelectedTrackSensorWorkbench } from "./SelectedTrackSensorWorkbench";
import { TrackSensorWorkbench } from "./TrackSensorWorkbench";
import {
  ACTIVE_TRACK_FIXTURE,
  NO_ADVISORY_TRACK_FIXTURE,
  STALE_TRACK_FIXTURE,
} from "./trackSensorWorkbenchFixtures";

describe("track sensor workbench UX polish", () => {
  it("renders compact selected track context header", () => {
    const markup = renderToStaticMarkup(<TrackSensorWorkbench model={ACTIVE_TRACK_FIXTURE} />);
    expect(markup).toContain('data-testid="track-context-header"');
    expect(markup).toContain("track_id");
    expect(markup).toContain("linked_entity_id");
    expect(markup).toContain("track_state");
    expect(markup).toContain("track_age");
    expect(markup).toContain("freshness");
    expect(markup).toContain("50 s");
    expect(markup).toContain("Fresh");
  });

  it("keeps stale track explanation sections visible", () => {
    const markup = renderToStaticMarkup(<TrackSensorWorkbench model={STALE_TRACK_FIXTURE} />);
    expect(markup).toContain('data-testid="track-stale-banner"');
    expect(markup).toContain("Track stale - explanation data preserved for review.");
    expect(markup).toContain('data-testid="track-detail-panel"');
    expect(markup).toContain('data-testid="sensor-contribution-panel"');
    expect(markup).toContain('data-testid="track-lifecycle-panel"');
    expect(markup).toContain('data-testid="track-confidence-panel"');
    expect(markup).toContain('data-testid="track-advisory-link-panel"');
    expect(markup).toContain("74 s");
  });

  it("renders polished empty state without placeholder data", () => {
    const markup = renderToStaticMarkup(<SelectedTrackSensorWorkbench selectedTrackId={null} />);
    expect(markup).toContain('data-testid="selected-track-sensor-empty"');
    expect(markup).toContain("Track &amp; sensor workbench");
    expect(markup).toContain("Select a track to inspect sensor and track details.");
    expect(markup).not.toContain("track-17");
    expect(markup).not.toContain("attacker-a");
  });

  it("keeps governance language visible in the sticky workbench header", () => {
    const markup = renderToStaticMarkup(<TrackSensorWorkbench model={ACTIVE_TRACK_FIXTURE} />);
    expect(markup).toContain("sticky top-0");
    expect(markup).toContain("read-only explanation only");
    expect(markup).toContain("no assignment, engagement, or autonomy authority");
  });

  it("renders active and no-advisory tracks after polish", () => {
    const activeMarkup = renderToStaticMarkup(<TrackSensorWorkbench model={ACTIVE_TRACK_FIXTURE} />);
    const noAdvisoryMarkup = renderToStaticMarkup(
      <TrackSensorWorkbench model={NO_ADVISORY_TRACK_FIXTURE} />,
    );
    expect(activeMarkup).toContain("track-17");
    expect(activeMarkup).toContain("defender-b");
    expect(noAdvisoryMarkup).toContain("track-31");
    expect(noAdvisoryMarkup).toContain("38 s");
    expect(noAdvisoryMarkup).toContain("No advisory link available for this track.");
  });
});
