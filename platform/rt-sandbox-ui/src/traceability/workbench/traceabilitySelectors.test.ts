import { describe, expect, it } from "vitest";
import { TRACEABILITY_FIXTURE_INPUTS } from "./traceabilityFixtureInputs";
import {
  MISMATCH_LINEAGE_FIXTURE,
  MISSING_ADVISORY_FIXTURE,
  PARTIAL_LINEAGE_FIXTURE,
  STALE_ADVISORY_FIXTURE,
} from "./traceabilityWorkbenchFixtures";
import {
  assembleTraceabilityWorkbenchModel,
  deriveFreshnessAlignment,
  resolveTraceabilityLinkage,
} from "./traceabilitySelectors";

describe("traceability selectors", () => {
  it("assembles full linked lineage from track and advisory inputs", () => {
    const model = assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.fullyLinked);

    expect(model.summary).toMatchObject({
      track_id: "track-17",
      attacker_id: "attacker-a",
      threat_rank: 1,
      threat_score: 48.2,
      advisory_id: "adv-attacker-a",
      recommended_defender: "defender-b",
      linkage_status: "linked",
      freshness_alignment: "track_and_advisory_fresh",
    });
    expect(model.track_lineage.attacker_id).toBe("attacker-a");
    expect(model.threat_lineage?.attacker_id).toBe("attacker-a");
    expect(model.threat_lineage?.threat_components).toHaveLength(4);
    expect(model.advisory_origin).toMatchObject({
      advisory_id: "adv-attacker-a",
      recommended_defender: "defender-b",
      defender_rank: 1,
      tti_s: 40,
      advisory_freshness: "fresh",
    });
  });

  it("assembles partial lineage when advisory exists without track advisory link", () => {
    const model = assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.partial);

    expect(model.summary).toMatchObject({
      track_id: "track-31",
      attacker_id: "attacker-c",
      advisory_id: null,
      recommended_defender: null,
      linkage_status: "partial",
      freshness_alignment: "track_fresh_advisory_missing",
    });
    expect(model.threat_lineage).toMatchObject({
      attacker_id: "attacker-c",
      threat_rank: 2,
      threat_score: 35.7,
    });
    expect(model.threat_lineage?.confidence_basis).toContain("advisory_gap");
    expect(model.advisory_origin).toMatchObject({
      advisory_id: null,
      attacker_id: "attacker-c",
      recommended_defender: null,
      advisory_freshness: "unknown",
    });
    expect(model.advisory_origin?.explanation).toContain("no advisory record is linked");
  });

  it("assembles missing advisory lineage without synthesizing threat or advisory origin", () => {
    const model = assembleTraceabilityWorkbenchModel(
      TRACEABILITY_FIXTURE_INPUTS.missingAdvisory,
    );

    expect(model.summary).toMatchObject({
      track_id: "track-42",
      attacker_id: "attacker-d",
      threat_rank: null,
      threat_score: null,
      advisory_id: null,
      recommended_defender: null,
      linkage_status: "missing",
      freshness_alignment: "advisory_unavailable",
    });
    expect(model.threat_lineage).toBeNull();
    expect(model.advisory_origin).toBeNull();
  });

  it("assembles stale lineage from stale track and stale advisory metadata", () => {
    const model = assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.staleAdvisory);

    expect(model.summary).toMatchObject({
      track_id: "track-23",
      attacker_id: "attacker-b",
      linkage_status: "stale",
      freshness_alignment: "track_stale_advisory_preserved",
    });
    expect(model.track_lineage.freshness).toBe("stale");
    expect(model.threat_lineage?.confidence_basis).toContain("stale_track_context");
    expect(model.advisory_origin).toMatchObject({
      advisory_id: "adv-attacker-b",
      recommended_defender: "defender-c",
      advisory_freshness: "stale",
      stale_reason: "track_stale_beyond_freshness_window",
    });
    expect(model.advisory_origin?.explanation).toContain("Advisory preserved from prior evaluation");
  });

  it("assembles mismatch lineage when linked entity and advisory attacker diverge", () => {
    const model = assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.mismatch);

    expect(model.summary).toMatchObject({
      track_id: "track-55",
      attacker_id: "attacker-e",
      advisory_id: "adv-attacker-f",
      linkage_status: "mismatch",
      freshness_alignment: "attacker_id_mismatch",
    });
    expect(model.threat_lineage?.attacker_id).toBe("attacker-e");
    expect(model.advisory_origin?.attacker_id).toBe("attacker-f");
    expect(model.threat_lineage?.confidence_basis).toContain("linkage_mismatch");
    expect(model.advisory_origin?.reason_codes).toContain("linkage_mismatch");
    expect(model.advisory_origin?.explanation).toContain("does not match track-linked attacker-e");
  });

  it("preserves no-recommendation advisory state in partial lineage", () => {
    const model = PARTIAL_LINEAGE_FIXTURE;
    expect(model.advisory_origin?.recommended_defender).toBeNull();
    expect(model.advisory_origin?.defender_rank).toBeNull();
    expect(model.advisory_origin?.tti_s).toBeNull();
    expect(model.advisory_origin?.reason_codes).toContain("no_solution");
  });

  it("derives freshness alignment tokens from local metadata only", () => {
    const linked = resolveTraceabilityLinkage(TRACEABILITY_FIXTURE_INPUTS.fullyLinked);
    expect(deriveFreshnessAlignment(TRACEABILITY_FIXTURE_INPUTS.fullyLinked, linked)).toBe(
      "track_and_advisory_fresh",
    );

    const partial = resolveTraceabilityLinkage(TRACEABILITY_FIXTURE_INPUTS.partial);
    expect(deriveFreshnessAlignment(TRACEABILITY_FIXTURE_INPUTS.partial, partial)).toBe(
      "track_fresh_advisory_missing",
    );

    const missing = resolveTraceabilityLinkage(TRACEABILITY_FIXTURE_INPUTS.missingAdvisory);
    expect(deriveFreshnessAlignment(TRACEABILITY_FIXTURE_INPUTS.missingAdvisory, missing)).toBe(
      "advisory_unavailable",
    );

    const stale = resolveTraceabilityLinkage(TRACEABILITY_FIXTURE_INPUTS.staleAdvisory);
    expect(deriveFreshnessAlignment(TRACEABILITY_FIXTURE_INPUTS.staleAdvisory, stale)).toBe(
      "track_stale_advisory_preserved",
    );

    const mismatch = resolveTraceabilityLinkage(TRACEABILITY_FIXTURE_INPUTS.mismatch);
    expect(deriveFreshnessAlignment(TRACEABILITY_FIXTURE_INPUTS.mismatch, mismatch)).toBe(
      "attacker_id_mismatch",
    );
  });

  it("matches golden fixture outputs for all selector-driven scenarios", () => {
    expect(assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.fullyLinked)).toEqual(
      assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.fullyLinked),
    );
    expect(assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.partial)).toEqual(
      PARTIAL_LINEAGE_FIXTURE,
    );
    expect(assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.missingAdvisory)).toEqual(
      MISSING_ADVISORY_FIXTURE,
    );
    expect(assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.staleAdvisory)).toEqual(
      STALE_ADVISORY_FIXTURE,
    );
    expect(assembleTraceabilityWorkbenchModel(TRACEABILITY_FIXTURE_INPUTS.mismatch)).toEqual(
      MISMATCH_LINEAGE_FIXTURE,
    );
  });
});
