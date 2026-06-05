import type { RtIntelligenceAdvisoryV1, ThreatComponentValue } from "@/intelligence/intelligenceAdvisory";
import type { TrackSensorWorkbenchModel } from "@/tracks/workbench/trackSensorWorkbenchTypes";
import { TRACEABILITY_FIXTURE_INPUTS } from "./traceabilityFixtureInputs";
import type {
  AdvisoryFreshness,
  AdvisoryOrigin,
  ConfidenceLevel,
  LinkageStatus,
  ThreatComponentRow,
  ThreatLineage,
  TraceabilitySummary,
  TraceabilityWorkbenchModel,
  TrackLineage,
} from "./traceabilityWorkbenchTypes";

export type TraceabilityFixtureMetadata = {
  advisory_stale?: boolean;
  advisory_stale_reason?: string | null;
};

export type TraceabilityAssemblyInput = {
  trackModel: TrackSensorWorkbenchModel;
  advisory: RtIntelligenceAdvisoryV1 | null;
  metadata?: TraceabilityFixtureMetadata;
};

type LinkageResolution = {
  status: LinkageStatus;
  entityMatch: boolean;
  hasAdvisoryLink: boolean;
  hasAdvisory: boolean;
  resolvedAttackerId: string | null;
};

const THREAT_COMPONENT_ROWS: Array<{
  key: keyof RtIntelligenceAdvisoryV1["threat_evaluation"]["threat_components"];
  label: string;
}> = [
  { key: "distance_to_protected_center", label: "Distance to protected center" },
  { key: "best_feasible_tti", label: "Best feasible TTI" },
  { key: "descent_factor", label: "Descent factor" },
  { key: "critical_zone_factor", label: "Critical zone factor" },
];

function finiteNumber(value: number | null | undefined): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function formatThreatComponentValue(component: ThreatComponentValue): string {
  const valueM = finiteNumber(component.value_m);
  if (valueM !== null) return `${valueM.toFixed(1)} m`;

  const valueS = finiteNumber(component.value_s);
  if (valueS !== null) return `${valueS.toFixed(1)} s`;

  const valueMps = finiteNumber(component.value_mps);
  if (valueMps !== null) return `${valueMps.toFixed(1)} m/s`;

  if (typeof component.active === "boolean") {
    return component.active ? "active" : "inactive";
  }

  return "-";
}

function mapConfidenceLevel(level: string | null | undefined): ConfidenceLevel {
  if (level === "high" || level === "medium" || level === "low") return level;
  return "unknown";
}

function entityIdsMatch(
  linkedEntityId: string | null,
  attackerId: string | null | undefined,
): boolean {
  if (!linkedEntityId || !attackerId) return false;
  return linkedEntityId === attackerId;
}

export function resolveTraceabilityLinkage(input: TraceabilityAssemblyInput): LinkageResolution {
  const { trackModel, advisory } = input;
  const linkedEntityId = trackModel.track.linked_entity_id;
  const advisoryLink = trackModel.advisory_link;
  const hasAdvisory = advisory !== null;
  const hasAdvisoryLink = advisoryLink !== null;
  const advisoryAttackerId = advisory?.identity.attacker_id ?? advisoryLink?.attacker_id ?? null;
  const entityMatch = entityIdsMatch(linkedEntityId, advisoryAttackerId);
  const resolvedAttackerId = linkedEntityId ?? advisoryAttackerId;

  if (!linkedEntityId && !hasAdvisory && !hasAdvisoryLink) {
    return {
      status: "missing",
      entityMatch: false,
      hasAdvisoryLink,
      hasAdvisory,
      resolvedAttackerId: null,
    };
  }

  if (hasAdvisory && linkedEntityId && !entityIdsMatch(linkedEntityId, advisory.identity.attacker_id)) {
    return {
      status: "mismatch",
      entityMatch: false,
      hasAdvisoryLink,
      hasAdvisory,
      resolvedAttackerId: linkedEntityId,
    };
  }

  if (!hasAdvisory && !hasAdvisoryLink) {
    return {
      status: "missing",
      entityMatch: false,
      hasAdvisoryLink,
      hasAdvisory,
      resolvedAttackerId: linkedEntityId,
    };
  }

  const trackStale = trackModel.track.staleness === "stale";
  const advisoryStale = input.metadata?.advisory_stale === true;
  if (trackStale || advisoryStale) {
    return {
      status: "stale",
      entityMatch,
      hasAdvisoryLink,
      hasAdvisory,
      resolvedAttackerId,
    };
  }

  if (!hasAdvisoryLink || (hasAdvisory && !hasAdvisoryLink)) {
    return {
      status: "partial",
      entityMatch,
      hasAdvisoryLink,
      hasAdvisory,
      resolvedAttackerId,
    };
  }

  if (entityMatch && hasAdvisoryLink) {
    return {
      status: "linked",
      entityMatch,
      hasAdvisoryLink,
      hasAdvisory,
      resolvedAttackerId,
    };
  }

  return {
    status: "partial",
    entityMatch,
    hasAdvisoryLink,
    hasAdvisory,
    resolvedAttackerId,
  };
}

export function deriveFreshnessAlignment(
  input: TraceabilityAssemblyInput,
  linkage: LinkageResolution,
): string {
  const trackFreshness = input.trackModel.track.staleness;
  const advisoryStale = input.metadata?.advisory_stale === true;
  const hasAdvisory = input.advisory !== null;
  const linkAlignment = input.trackModel.advisory_link?.freshness_alignment;

  if (linkage.status === "mismatch") return "attacker_id_mismatch";
  if (linkage.status === "missing") return "advisory_unavailable";
  if (linkage.status === "partial") return "track_fresh_advisory_missing";

  if (linkage.status === "stale" && linkAlignment) return linkAlignment;
  if (trackFreshness === "stale" && advisoryStale) return "both_stale";
  if (trackFreshness === "stale") return "track_stale";
  if (advisoryStale) return "advisory_stale";
  if (trackFreshness === "unknown") return "unknown";

  if (linkage.status === "linked" && hasAdvisory && !advisoryStale) {
    return linkAlignment ?? "fresh";
  }

  return "unknown";
}

function buildThreatComponents(advisory: RtIntelligenceAdvisoryV1): ThreatComponentRow[] {
  const components = advisory.threat_evaluation.threat_components;
  return THREAT_COMPONENT_ROWS.map(({ key, label }) => {
    const component = components[key];
    return {
      key,
      label,
      value_display: formatThreatComponentValue(component),
      normalized: finiteNumber(component.normalized),
      weight: finiteNumber(component.weight),
    };
  });
}

function appendLinkageBasis(
  basis: string[],
  linkage: LinkageResolution,
  trackStale: boolean,
): string[] {
  const next = [...basis];
  if (linkage.status === "partial" && !next.includes("advisory_gap")) {
    next.push("advisory_gap");
  }
  if (linkage.status === "mismatch" && !next.includes("linkage_mismatch")) {
    next.push("linkage_mismatch");
  }
  if (trackStale && !next.includes("stale_track_context")) {
    next.push("stale_track_context");
  }
  return next;
}

function buildThreatLineage(
  input: TraceabilityAssemblyInput,
  linkage: LinkageResolution,
): ThreatLineage | null {
  const { advisory, trackModel } = input;
  if (!advisory || linkage.status === "missing") return null;

  const trackLinkedAttackerId = trackModel.track.linked_entity_id;
  const attackerId =
    linkage.status === "mismatch"
      ? trackLinkedAttackerId ?? advisory.identity.attacker_id
      : advisory.identity.attacker_id;

  if (!attackerId) return null;

  const heuristic = advisory.confidence.heuristic_confidence;
  return {
    attacker_id: attackerId,
    threat_rank: advisory.threat_evaluation.threat_rank,
    threat_score: advisory.threat_evaluation.threat_score,
    threat_components: buildThreatComponents(advisory),
    heuristic_confidence_level: mapConfidenceLevel(heuristic.level),
    heuristic_confidence_score: finiteNumber(heuristic.score),
    confidence_basis: appendLinkageBasis(
      [...heuristic.basis],
      linkage,
      trackModel.track.staleness === "stale",
    ),
  };
}

function defenderRank(advisory: RtIntelligenceAdvisoryV1): number | null {
  const defenderId = advisory.recommended_defender.defender_id;
  if (!defenderId) return null;
  const row = advisory.defender_ranking.ranked_defenders.find(
    (candidate) => candidate.defender_id === defenderId,
  );
  return row?.rank ?? null;
}

function buildAdvisoryOrigin(
  input: TraceabilityAssemblyInput,
  linkage: LinkageResolution,
): AdvisoryOrigin | null {
  const { advisory, metadata } = input;
  if (!advisory || linkage.status === "missing") return null;

  const advisoryStale = metadata?.advisory_stale === true;
  const unlinked = linkage.status === "partial" || !linkage.hasAdvisoryLink;
  const noRecommendation =
    advisory.recommended_defender.defender_id === null ||
    !advisory.recommended_defender.feasibility.feasible;

  let advisoryFreshness: AdvisoryFreshness = "fresh";
  if (advisoryStale) advisoryFreshness = "stale";
  else if (unlinked) advisoryFreshness = "unknown";

  let explanation = advisory.reasoning.explanation;
  if (linkage.status === "mismatch" && trackModelLinkedAttacker(input)) {
    explanation = `Advisory ${advisory.identity.attacker_id} does not match track-linked ${trackModelLinkedAttacker(input)}; review as explanatory correlation only.`;
  } else if (unlinked && linkage.status === "partial") {
    explanation = "Threat evaluation available, but no advisory record is linked to this track.";
  } else if (advisoryStale) {
    explanation = "Advisory preserved from prior evaluation while track state is stale.";
  }

  const reasonCodes =
    linkage.status === "mismatch"
      ? [...advisory.reasoning.reason_codes, "linkage_mismatch"]
      : advisory.reasoning.reason_codes;

  return {
    advisory_id: unlinked ? null : advisory.identity.advisory_id,
    attacker_id: advisory.identity.attacker_id,
    recommended_defender: advisory.recommended_defender.defender_id,
    defender_rank: noRecommendation ? null : defenderRank(advisory),
    tti_s: noRecommendation ? null : advisory.recommended_defender.tti_s,
    reason_codes: reasonCodes,
    explanation,
    advisory_freshness: advisoryFreshness,
    advisory_utc: unlinked ? null : advisory.identity.advisory_utc,
    stale_reason: advisoryStale ? metadata?.advisory_stale_reason ?? null : null,
  };
}

function trackModelLinkedAttacker(input: TraceabilityAssemblyInput): string | null {
  return input.trackModel.track.linked_entity_id;
}

function buildTrackLineage(
  input: TraceabilityAssemblyInput,
  linkage: LinkageResolution,
): TrackLineage {
  const { track, confidence } = input.trackModel;
  return {
    track_id: track.track_id,
    linked_entity_id: track.linked_entity_id,
    attacker_id: linkage.resolvedAttackerId,
    track_state: track.track_state,
    track_age_s: track.track_age_s,
    freshness: track.staleness,
    confidence_level: mapConfidenceLevel(confidence.level),
    confidence_score: finiteNumber(confidence.score),
    confidence_basis: confidence.basis,
  };
}

function buildSummary(
  input: TraceabilityAssemblyInput,
  linkage: LinkageResolution,
  freshnessAlignment: string,
): TraceabilitySummary {
  const { trackModel, advisory } = input;
  const advisoryLink = trackModel.advisory_link;

  return {
    track_id: trackModel.track.track_id,
    attacker_id: linkage.resolvedAttackerId,
    threat_rank: advisory?.threat_evaluation.threat_rank ?? advisoryLink?.threat_rank ?? null,
    threat_score: advisory?.threat_evaluation.threat_score ?? advisoryLink?.threat_score ?? null,
    advisory_id:
      linkage.status === "partial" || linkage.status === "missing"
        ? null
        : advisory?.identity.advisory_id ?? null,
    recommended_defender:
      advisory?.recommended_defender.defender_id ?? advisoryLink?.recommended_defender ?? null,
    linkage_status: linkage.status,
    freshness_alignment: freshnessAlignment,
  };
}

export function assembleTraceabilityWorkbenchModel(
  input: TraceabilityAssemblyInput,
): TraceabilityWorkbenchModel {
  const linkage = resolveTraceabilityLinkage(input);
  const freshnessAlignment = deriveFreshnessAlignment(input, linkage);

  return {
    summary: buildSummary(input, linkage, freshnessAlignment),
    track_lineage: buildTrackLineage(input, linkage),
    threat_lineage: buildThreatLineage(input, linkage),
    advisory_origin: buildAdvisoryOrigin(input, linkage),
  };
}

export function listTraceabilityFixtureInputs(
  inputs: Record<string, TraceabilityAssemblyInput> = TRACEABILITY_FIXTURE_INPUTS,
): readonly TraceabilityAssemblyInput[] {
  return Object.values(inputs);
}

export function getTraceabilityAssemblyInputForTrack(
  selectedTrackId: string | null,
  inputs: Record<string, TraceabilityAssemblyInput> = TRACEABILITY_FIXTURE_INPUTS,
): TraceabilityAssemblyInput | null {
  if (selectedTrackId === null || selectedTrackId.trim().length === 0) return null;
  return (
    listTraceabilityFixtureInputs(inputs).find(
      (input) => input.trackModel.track.track_id === selectedTrackId,
    ) ?? null
  );
}

export function getSelectedTraceabilityModel(
  selectedTrackId: string | null,
  inputs: Record<string, TraceabilityAssemblyInput> = TRACEABILITY_FIXTURE_INPUTS,
): TraceabilityWorkbenchModel | null {
  const input = getTraceabilityAssemblyInputForTrack(selectedTrackId, inputs);
  return input === null ? null : assembleTraceabilityWorkbenchModel(input);
}
