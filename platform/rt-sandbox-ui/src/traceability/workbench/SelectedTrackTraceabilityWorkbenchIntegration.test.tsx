import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { SelectedTrackTraceabilityWorkbench } from "./SelectedTrackTraceabilityWorkbench";
import {
  MISMATCH_LINEAGE_FIXTURE,
  MISSING_ADVISORY_FIXTURE,
  PARTIAL_LINEAGE_FIXTURE,
  STALE_ADVISORY_FIXTURE,
} from "./traceabilityWorkbenchFixtures";
import {
  getSelectedTraceabilityModel,
  getTraceabilityAssemblyInputForTrack,
  listTraceabilityFixtureInputs,
} from "./traceabilitySelectors";

describe("selected track traceability workbench integration", () => {
  it("binds fixture inputs by selected track id without telemetry", () => {
    expect(listTraceabilityFixtureInputs()).toHaveLength(5);
    expect(getTraceabilityAssemblyInputForTrack("track-17")?.trackModel.track.track_id).toBe(
      "track-17",
    );
    expect(getTraceabilityAssemblyInputForTrack("track-31")?.trackModel.track.track_id).toBe(
      "track-31",
    );
    expect(getTraceabilityAssemblyInputForTrack("track-42")?.trackModel.track.track_id).toBe(
      "track-42",
    );
    expect(getTraceabilityAssemblyInputForTrack("track-23")?.trackModel.track.track_id).toBe(
      "track-23",
    );
    expect(getTraceabilityAssemblyInputForTrack("track-55")?.trackModel.track.track_id).toBe(
      "track-55",
    );
    expect(getTraceabilityAssemblyInputForTrack("missing-track")).toBeNull();
    expect(getSelectedTraceabilityModel(null)).toBeNull();
  });

  it("renders selected linked track with context header and governance copy", () => {
    const markup = renderToStaticMarkup(
      <SelectedTrackTraceabilityWorkbench selectedTrackId="track-17" />,
    );
    expect(markup).toContain('data-testid="selected-track-traceability-workbench"');
    expect(markup).toContain('data-testid="traceability-context-header"');
    expect(markup).toContain('data-testid="traceability-workbench"');
    expect(markup).toContain("track-17");
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("Linked");
    expect(markup).toContain("Track and advisory fresh");
    expect(markup).toContain("Explanatory lineage only");
    expect(markup).toContain("Recommendation origin visibility only");
    expect(markup).toContain("No assignment authority");
    expect(markup).toContain("No engagement authority");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
  });

  it("switches selected track rendering", () => {
    const activeMarkup = renderToStaticMarkup(
      <SelectedTrackTraceabilityWorkbench selectedTrackId="track-17" />,
    );
    const partialMarkup = renderToStaticMarkup(
      <SelectedTrackTraceabilityWorkbench selectedTrackId="track-31" />,
    );
    expect(activeMarkup).toContain("track-17");
    expect(activeMarkup).not.toContain("track-31");
    expect(partialMarkup).toContain("track-31");
    expect(partialMarkup).not.toContain("track-17");
    expect(partialMarkup).toContain("Partial");
  });

  it("renders stale lineage with preserved sections and context header", () => {
    const markup = renderToStaticMarkup(
      <SelectedTrackTraceabilityWorkbench selectedTrackId="track-23" />,
    );
    expect(markup).toContain("track-23");
    expect(markup).toContain("Stale");
    expect(markup).toContain("Track stale advisory preserved");
    expect(markup).toContain('data-testid="traceability-stale-banner"');
    expect(markup).toContain('data-testid="track-lineage-panel"');
    expect(markup).toContain('data-testid="threat-lineage-panel"');
    expect(markup).toContain('data-testid="advisory-origin-panel"');
    expect(markup).toContain("track_stale_beyond_freshness_window");
    expect(getSelectedTraceabilityModel("track-23")).toEqual(STALE_ADVISORY_FIXTURE);
  });

  it("renders partial lineage without hiding lineage sections", () => {
    const markup = renderToStaticMarkup(
      <SelectedTrackTraceabilityWorkbench selectedTrackId="track-31" />,
    );
    expect(markup).toContain("Partial");
    expect(markup).toContain("Track fresh advisory missing");
    expect(markup).toContain('data-testid="track-lineage-panel"');
    expect(markup).toContain('data-testid="threat-lineage-panel"');
    expect(markup).toContain('data-testid="advisory-origin-panel"');
    expect(markup).toContain("Partial advisory linkage");
    expect(markup).toContain("No feasible defender recommendation available.");
    expect(getSelectedTraceabilityModel("track-31")).toEqual(PARTIAL_LINEAGE_FIXTURE);
  });

  it("renders mismatch lineage with divergence visibility", () => {
    const markup = renderToStaticMarkup(
      <SelectedTrackTraceabilityWorkbench selectedTrackId="track-55" />,
    );
    expect(markup).toContain("Mismatch");
    expect(markup).toContain("attacker-e");
    expect(markup).toContain("attacker-f");
    expect(markup).toContain('data-testid="traceability-mismatch-banner"');
    expect(markup).toContain('data-testid="track-lineage-panel"');
    expect(markup).toContain('data-testid="threat-lineage-panel"');
    expect(markup).toContain('data-testid="advisory-origin-panel"');
    expect(getSelectedTraceabilityModel("track-55")).toEqual(MISMATCH_LINEAGE_FIXTURE);
  });

  it("renders missing advisory lineage without placeholder lineage data", () => {
    const markup = renderToStaticMarkup(
      <SelectedTrackTraceabilityWorkbench selectedTrackId="track-42" />,
    );
    expect(markup).toContain("Missing");
    expect(markup).toContain("Advisory unavailable");
    expect(markup).toContain('data-testid="threat-lineage-missing"');
    expect(markup).toContain('data-testid="advisory-origin-missing"');
    expect(markup).not.toContain("defender-b");
    expect(getSelectedTraceabilityModel("track-42")).toEqual(MISSING_ADVISORY_FIXTURE);
  });

  it("renders empty selected-track state without placeholder lineage", () => {
    const markup = renderToStaticMarkup(
      <SelectedTrackTraceabilityWorkbench selectedTrackId={null} />,
    );
    expect(markup).toContain('data-testid="selected-track-traceability-empty"');
    expect(markup).toContain("Select a track to inspect lineage and recommendation origin.");
    expect(markup).not.toContain('data-testid="traceability-workbench"');
    expect(markup).not.toContain('data-testid="track-lineage-panel"');
    expect(markup).not.toContain("track-17");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
  });
});
