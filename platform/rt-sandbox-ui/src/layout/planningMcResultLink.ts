/**
 * UI-local Planning ↔ MC result linkage (metadata only).
 * No MC execution, filesystem access, runtime, or bridge mutation.
 */

import type { PlanningMcPackageV1 } from "./planningMcPackage";
import { planningPackageLinkId } from "./planningMcPackage";

export const PLANNING_MC_RESULT_REF_SCHEMA_VERSION =
  "rt_planning_mc_result_ref_v1" as const;
export const PLANNING_RESULT_LINK_SCHEMA_VERSION = "planning_result_link_v1" as const;

export type PlanningResultLinkStatus = "unlinked" | "linked" | "stale" | "mismatch";

export type PlanningMcResultSummaryV1 = {
  success_rate?: number;
  miss_distance_p95?: number;
  intercept_time_mean?: number;
};

export type PlanningMcResultRefV1 = {
  schema_version: typeof PLANNING_MC_RESULT_REF_SCHEMA_VERSION;
  linked_package_id: string;
  linked_planning_geometry_id: string;
  mc_run_label: string;
  mc_result_id: string;
  imported_utc: string;
  summary?: PlanningMcResultSummaryV1;
};

export type PlanningResultLinkV1 = {
  schema_version: typeof PLANNING_RESULT_LINK_SCHEMA_VERSION;
  linked_package_id: string;
  linked_planning_snapshot_id: string;
  linked_planning_geometry_id: string;
  linked_source_layout_id?: string;
  linked_source_geometry_id?: string;
  result_ref: PlanningMcResultRefV1 | null;
  status: PlanningResultLinkStatus;
};

export type PlanningResultLinkValidation = {
  status: PlanningResultLinkStatus;
  reasons: string[];
};

export type ParsePlanningMcResultRefResult =
  | { ok: true; ref: PlanningMcResultRefV1 }
  | { ok: false; error: string };

function utcNow(): string {
  return new Date().toISOString().replace(/\.\d{3}Z$/, "Z");
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

function nonEmptyString(value: unknown): string | null {
  if (typeof value !== "string" || value.trim().length === 0) {
    return null;
  }
  return value.trim();
}

function optionalFiniteNumber(value: unknown): number | undefined {
  if (value === undefined || value === null) return undefined;
  if (typeof value !== "number" || !Number.isFinite(value)) return undefined;
  return value;
}

function parseSummary(value: unknown): PlanningMcResultSummaryV1 | undefined {
  if (!isRecord(value)) return undefined;
  const summary: PlanningMcResultSummaryV1 = {};
  const successRate = optionalFiniteNumber(value.success_rate);
  const missDistanceP95 = optionalFiniteNumber(value.miss_distance_p95);
  const interceptTimeMean = optionalFiniteNumber(value.intercept_time_mean);
  if (successRate !== undefined) summary.success_rate = successRate;
  if (missDistanceP95 !== undefined) summary.miss_distance_p95 = missDistanceP95;
  if (interceptTimeMean !== undefined) summary.intercept_time_mean = interceptTimeMean;
  return Object.keys(summary).length > 0 ? summary : undefined;
}

export function parsePlanningMcResultRefJson(text: string): ParsePlanningMcResultRefResult {
  let parsed: unknown;
  try {
    parsed = JSON.parse(text);
  } catch {
    return { ok: false, error: "Invalid JSON" };
  }
  if (!isRecord(parsed)) {
    return { ok: false, error: "Expected JSON object" };
  }
  if (parsed.schema_version !== PLANNING_MC_RESULT_REF_SCHEMA_VERSION) {
    return {
      ok: false,
      error: `expected schema_version ${PLANNING_MC_RESULT_REF_SCHEMA_VERSION}`,
    };
  }
  const linkedPackageId = nonEmptyString(parsed.linked_package_id);
  const linkedPlanningGeometryId = nonEmptyString(parsed.linked_planning_geometry_id);
  const mcRunLabel = nonEmptyString(parsed.mc_run_label);
  const mcResultId = nonEmptyString(parsed.mc_result_id);
  const importedUtc = nonEmptyString(parsed.imported_utc);
  if (
    !linkedPackageId ||
    !linkedPlanningGeometryId ||
    !mcRunLabel ||
    !mcResultId ||
    !importedUtc
  ) {
    return { ok: false, error: "Missing required result reference fields" };
  }
  const summary = parseSummary(parsed.summary);
  return {
    ok: true,
    ref: {
      schema_version: PLANNING_MC_RESULT_REF_SCHEMA_VERSION,
      linked_package_id: linkedPackageId,
      linked_planning_geometry_id: linkedPlanningGeometryId,
      mc_run_label: mcRunLabel,
      mc_result_id: mcResultId,
      imported_utc: importedUtc,
      ...(summary ? { summary } : {}),
    },
  };
}

export function buildMockPlanningMcResultRef(
  pkg: PlanningMcPackageV1,
  options: {
    mcRunLabel?: string;
    mcResultId?: string;
    importedUtc?: string;
    summary?: PlanningMcResultSummaryV1;
  } = {},
): PlanningMcResultRefV1 {
  return {
    schema_version: PLANNING_MC_RESULT_REF_SCHEMA_VERSION,
    linked_package_id: planningPackageLinkId(pkg),
    linked_planning_geometry_id: pkg.planning_geometry_id,
    mc_run_label: options.mcRunLabel ?? pkg.mc_preparation.scenario_label,
    mc_result_id: options.mcResultId ?? "rt_mc_result:mock:planning-ui",
    imported_utc: options.importedUtc ?? utcNow(),
    ...(options.summary ? { summary: options.summary } : {}),
  };
}

export function validatePlanningResultLink(
  pkg: PlanningMcPackageV1,
  resultRef: PlanningMcResultRefV1 | null,
  currentPlanningGeometryId: string,
): PlanningResultLinkValidation {
  if (!resultRef) {
    return { status: "unlinked", reasons: ["No MC result reference imported"] };
  }

  const reasons: string[] = [];
  const expectedPackageId = planningPackageLinkId(pkg);

  if (resultRef.linked_package_id !== expectedPackageId) {
    reasons.push("linked_package_id does not match current package");
  }
  if (resultRef.linked_planning_geometry_id !== pkg.planning_geometry_id) {
    reasons.push("linked_planning_geometry_id does not match package geometry");
  }
  if (reasons.length > 0) {
    return { status: "mismatch", reasons };
  }

  if (
    pkg.planning_geometry_id !== currentPlanningGeometryId ||
    resultRef.linked_planning_geometry_id !== currentPlanningGeometryId
  ) {
    return {
      status: "stale",
      reasons: ["Planning geometry changed since result import"],
    };
  }

  return { status: "linked", reasons: [] };
}

export function buildPlanningResultLink(
  pkg: PlanningMcPackageV1,
  resultRef: PlanningMcResultRefV1 | null,
  currentPlanningGeometryId: string,
): PlanningResultLinkV1 {
  const validation = validatePlanningResultLink(pkg, resultRef, currentPlanningGeometryId);
  return {
    schema_version: PLANNING_RESULT_LINK_SCHEMA_VERSION,
    linked_package_id: planningPackageLinkId(pkg),
    linked_planning_snapshot_id: pkg.planning_snapshot_id,
    linked_planning_geometry_id: pkg.planning_geometry_id,
    ...(pkg.source_layout_id ? { linked_source_layout_id: pkg.source_layout_id } : {}),
    ...(pkg.source_geometry_id ? { linked_source_geometry_id: pkg.source_geometry_id } : {}),
    result_ref: resultRef,
    status: validation.status,
  };
}

export function buildEmptyPlanningResultLink(pkg: PlanningMcPackageV1): PlanningResultLinkV1 {
  return buildPlanningResultLink(pkg, null, pkg.planning_geometry_id);
}

export type PlanningResultLinkPreview = {
  status: PlanningResultLinkStatus;
  statusLabel: string;
  reasons: string[];
  mcRunLabel: string | null;
  mcResultId: string | null;
  importedUtc: string | null;
  successRate: number | null;
  missDistanceP95: number | null;
  interceptTimeMean: number | null;
};

export function buildPlanningResultLinkPreview(
  link: PlanningResultLinkV1 | null,
): PlanningResultLinkPreview {
  if (!link) {
    return {
      status: "unlinked",
      statusLabel: "Unlinked",
      reasons: ["No Planning MC package"],
      mcRunLabel: null,
      mcResultId: null,
      importedUtc: null,
      successRate: null,
      missDistanceP95: null,
      interceptTimeMean: null,
    };
  }

  const ref = link.result_ref;
  const validation =
    ref === null
      ? { status: "unlinked" as const, reasons: ["No MC result reference imported"] }
      : { status: link.status, reasons: [] };

  return {
    status: link.status,
    statusLabel:
      link.status === "linked"
        ? "Linked"
        : link.status === "stale"
          ? "Stale"
          : link.status === "mismatch"
            ? "Mismatch"
            : "Unlinked",
    reasons: validation.reasons,
    mcRunLabel: ref?.mc_run_label ?? null,
    mcResultId: ref?.mc_result_id ?? null,
    importedUtc: ref?.imported_utc ?? null,
    successRate: ref?.summary?.success_rate ?? null,
    missDistanceP95: ref?.summary?.miss_distance_p95 ?? null,
    interceptTimeMean: ref?.summary?.intercept_time_mean ?? null,
  };
}

export function exportPlanningMcResultRefJson(ref: PlanningMcResultRefV1): string {
  if (ref.schema_version !== PLANNING_MC_RESULT_REF_SCHEMA_VERSION) {
    throw new Error(`expected schema_version ${PLANNING_MC_RESULT_REF_SCHEMA_VERSION}`);
  }
  return `${JSON.stringify(ref, null, 2)}\n`;
}
