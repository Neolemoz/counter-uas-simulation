import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { AdvisoryOriginPanel } from "./AdvisoryOriginPanel";
import { ThreatLineagePanel } from "./ThreatLineagePanel";
import { TrackLineagePanel } from "./TrackLineagePanel";
import { TraceabilitySummaryPanel } from "./TraceabilitySummaryPanel";
import { TraceabilityWorkbench } from "./TraceabilityWorkbench";
import {
  FULLY_LINKED_LINEAGE_FIXTURE,
  MISMATCH_LINEAGE_FIXTURE,
  MISSING_ADVISORY_FIXTURE,
  PARTIAL_LINEAGE_FIXTURE,
  STALE_ADVISORY_FIXTURE,
} from "./traceabilityWorkbenchFixtures";
import { TRACEABILITY_WORKBENCH_GOVERNANCE } from "./traceabilityGovernance";

describe("traceability workbench", () => {
  it("renders full lineage without command controls", () => {
    const markup = renderToStaticMarkup(
      <TraceabilityWorkbench model={FULLY_LINKED_LINEAGE_FIXTURE} />,
    );
    expect(markup).toContain('data-testid="traceability-workbench"');
    expect(markup).toContain("Threat traceability workbench");
    expect(markup).toContain("track-17");
    expect(markup).toContain("attacker-a");
    expect(markup).toContain("adv-attacker-a");
    expect(markup).toContain("defender-b");
    expect(markup).toContain("Linked");
    expect(markup).toContain("Track and advisory fresh");
    expect(markup).toContain("Distance to protected center");
    expect(markup).toContain("descending fast");
    expect(markup).not.toMatch(/<button\b/);
    expect(markup).not.toMatch(/type="submit"/);
  });

  it("renders partial lineage with advisory gap cues", () => {
    const markup = renderToStaticMarkup(
      <TraceabilityWorkbench model={PARTIAL_LINEAGE_FIXTURE} />,
    );
    expect(markup).toContain("Partial");
    expect(markup).toContain("track-31");
    expect(markup).toContain("attacker-c");
    expect(markup).toContain("Track fresh advisory missing");
    expect(markup).toContain("No feasible defender recommendation available.");
    expect(markup).toContain("Partial advisory linkage");
    expect(markup).toContain("no advisory record is linked");
  });

  it("renders stale lineage with preserved explanation data", () => {
    const markup = renderToStaticMarkup(
      <TraceabilityWorkbench model={STALE_ADVISORY_FIXTURE} />,
    );
    expect(markup).toContain('data-testid="traceability-stale-banner"');
    expect(markup).toContain("track-23");
    expect(markup).toContain("coasting");
    expect(markup).toContain("Stale");
    expect(markup).toContain("track_stale_beyond_freshness_window");
    expect(markup).toContain("Track stale advisory preserved");
    expect(markup).toContain("Advisory preserved from prior evaluation");
  });

  it("renders missing advisory state", () => {
    const markup = renderToStaticMarkup(
      <TraceabilityWorkbench model={MISSING_ADVISORY_FIXTURE} />,
    );
    expect(markup).toContain("Missing");
    expect(markup).toContain("track-42");
    expect(markup).toContain("Advisory unavailable");
    expect(markup).toContain('data-testid="threat-lineage-missing"');
    expect(markup).toContain("No threat evaluation lineage available");
    expect(markup).toContain('data-testid="advisory-origin-missing"');
    expect(markup).toContain("No advisory origin available");
  });

  it("renders mismatch lineage with divergence banner", () => {
    const markup = renderToStaticMarkup(
      <TraceabilityWorkbench model={MISMATCH_LINEAGE_FIXTURE} />,
    );
    expect(markup).toContain('data-testid="traceability-mismatch-banner"');
    expect(markup).toContain("Mismatch");
    expect(markup).toContain("attacker-e");
    expect(markup).toContain("attacker-f");
    expect(markup).toContain("Attacker id mismatch");
    expect(markup).toContain("does not match track-linked attacker-e");
  });

  it("renders governance visibility across panels", () => {
    const markup = renderToStaticMarkup(
      <TraceabilityWorkbench model={FULLY_LINKED_LINEAGE_FIXTURE} />,
    );
    expect(markup).toContain(TRACEABILITY_WORKBENCH_GOVERNANCE);
    expect(markup).toContain("read-only lineage explanation only");
    expect(markup).toContain("Not mission success confidence");
    expect(markup).toContain("Not kill probability");
    expect(markup).toContain("does not create a control path");
  });

  it("renders summary panel linkage status tones", () => {
    const linked = renderToStaticMarkup(
      <TraceabilitySummaryPanel summary={FULLY_LINKED_LINEAGE_FIXTURE.summary} />,
    );
    expect(linked).toContain('data-testid="traceability-linkage-status"');
    expect(linked).toContain("Linked");

    const partial = renderToStaticMarkup(
      <TraceabilitySummaryPanel summary={PARTIAL_LINEAGE_FIXTURE.summary} />,
    );
    expect(partial).toContain("Partial");

    const mismatch = renderToStaticMarkup(
      <TraceabilitySummaryPanel summary={MISMATCH_LINEAGE_FIXTURE.summary} />,
    );
    expect(mismatch).toContain("Mismatch");
  });

  it("renders track lineage flow and confidence fields", () => {
    const markup = renderToStaticMarkup(
      <TrackLineagePanel lineage={FULLY_LINKED_LINEAGE_FIXTURE.track_lineage} />,
    );
    expect(markup).toContain("Track");
    expect(markup).toContain("Entity");
    expect(markup).toContain("Attacker");
    expect(markup).toContain("linked_entity_id");
    expect(markup).toContain("0.81");
    expect(markup).toContain("high");
    expect(markup).toContain("50 s");
  });

  it("renders threat lineage components and basis chips", () => {
    const markup = renderToStaticMarkup(
      <ThreatLineagePanel lineage={FULLY_LINKED_LINEAGE_FIXTURE.threat_lineage} />,
    );
    expect(markup).toContain("Threat Evaluation");
    expect(markup).toContain("48.2");
    expect(markup).toContain("#1");
    expect(markup).toContain("0.85");
    expect(markup).toContain("Complete attacker identity");
    expect(markup).toContain("Deterministic defender ranking");
  });

  it("renders advisory origin reason codes and no-recommendation state", () => {
    const markup = renderToStaticMarkup(
      <AdvisoryOriginPanel origin={PARTIAL_LINEAGE_FIXTURE.advisory_origin} />,
    );
    expect(markup).toContain("Inside warning ring");
    expect(markup).toContain("No solution");
    expect(markup).toContain("No feasible defender recommendation available.");
  });
});
