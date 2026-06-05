import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { TrackTraceabilityWorkstationSurface } from "@/workstation/AppWorkstationSlots";

describe("track traceability workstation placement", () => {
  it("renders standalone workbench surface in workstation", () => {
    const markup = renderToStaticMarkup(
      <TrackTraceabilityWorkstationSurface selectedTrackId="track-17" />,
    );
    expect(markup).toContain('data-testid="track-traceability-workstation-surface"');
    expect(markup).toContain('data-testid="selected-track-traceability-workbench"');
    expect(markup).toContain('data-testid="traceability-workbench"');
    expect(markup).toContain("Threat traceability workbench");
    expect(markup).toContain("THREAT TRACEABILITY");
  });

  it("renders empty state without auto-selecting tracks", () => {
    const markup = renderToStaticMarkup(
      <TrackTraceabilityWorkstationSurface selectedTrackId={null} />,
    );
    expect(markup).toContain('data-testid="selected-track-traceability-empty"');
    expect(markup).toContain("Select a track to inspect lineage and recommendation origin.");
    expect(markup).not.toContain('data-testid="traceability-workbench"');
    expect(markup).not.toContain("track-17");
    expect(markup).not.toContain("attacker-a");
  });

  it("renders selected track lineage", () => {
    const markup = renderToStaticMarkup(
      <TrackTraceabilityWorkstationSurface selectedTrackId="track-17" />,
    );
    expect(markup).toContain('data-testid="traceability-context-header"');
    expect(markup).toContain("track-17");
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("Linked");
    expect(markup).toContain("Track and advisory fresh");
    expect(markup).toContain('data-testid="track-lineage-panel"');
    expect(markup).toContain('data-testid="threat-lineage-panel"');
    expect(markup).toContain('data-testid="advisory-origin-panel"');
  });

  it("updates traceability view when selected track changes", () => {
    const activeMarkup = renderToStaticMarkup(
      <TrackTraceabilityWorkstationSurface selectedTrackId="track-17" />,
    );
    const partialMarkup = renderToStaticMarkup(
      <TrackTraceabilityWorkstationSurface selectedTrackId="track-31" />,
    );
    expect(activeMarkup).toContain("track-17");
    expect(activeMarkup).toContain("Linked");
    expect(activeMarkup).not.toContain("track-31");
    expect(partialMarkup).toContain("track-31");
    expect(partialMarkup).toContain("Partial");
    expect(partialMarkup).not.toContain("track-17");
  });

  it("keeps stale lineage visible with preserved sections", () => {
    const markup = renderToStaticMarkup(
      <TrackTraceabilityWorkstationSurface selectedTrackId="track-23" />,
    );
    expect(markup).toContain('data-testid="traceability-stale-banner"');
    expect(markup).toContain("Lineage stale - explanation data preserved for review.");
    expect(markup).toContain('data-testid="track-lineage-panel"');
    expect(markup).toContain('data-testid="threat-lineage-panel"');
    expect(markup).toContain('data-testid="advisory-origin-panel"');
    expect(markup).toContain("Track stale advisory preserved");
    expect(markup).toContain("track_stale_beyond_freshness_window");
  });

  it("keeps governance visibility and read-only enforcement", () => {
    const markup = renderToStaticMarkup(
      <TrackTraceabilityWorkstationSurface selectedTrackId="track-17" />,
    );
    expect(markup).toContain("Explanatory lineage only");
    expect(markup).toContain("Recommendation origin visibility only");
    expect(markup).toContain("No assignment authority");
    expect(markup).toContain("No engagement authority");
    expect(markup).toContain("read-only lineage explanation only");
    expect(markup).toContain("no assignment, engagement, or autonomy authority");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
    expect(markup).not.toContain("approve");
    expect(markup).not.toContain("reject");
    expect(markup).not.toContain("Assign");
  });
});
