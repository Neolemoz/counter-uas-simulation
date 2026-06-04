/**
 * UI-local Planning -> MC package preview.
 * Packaging only: no MC jobs, no execution, no runtime/bridge mutation.
 */

import { exportJson, copyTextToClipboard, type ClipboardResult } from "./layoutMcHandoff";
import type { PlanningMcSnapshotExtent, PlanningMcSnapshotV1 } from "./planningMcSnapshot";

export const PLANNING_MC_PACKAGE_SCHEMA_VERSION = "rt_planning_mc_package_v1" as const;
export const PLANNING_MC_PACKAGE_VERSION = "1" as const;

export {
  PLANNING_RESULT_LINK_SCHEMA_VERSION,
  buildEmptyPlanningResultLink,
  type PlanningResultLinkStatus,
  type PlanningResultLinkV1,
} from "./planningMcResultLink";
export const DEFAULT_PLANNING_MC_SCENARIO_LABEL = "planning-analysis" as const;
export const DEFAULT_PLANNING_MC_RUN_COUNT = 50;
export const DEFAULT_PLANNING_MC_SEED_BASE = 1;

export type PlanningMcPackageCoverageSummary = {
  coverage_percent: number;
  blind_spot_summary: string;
};

export type PlanningMcPackageOverlapSummary = {
  overlap_percent: number;
};

export type PlanningMcPackageRedundancySummary = {
  redundancy_percent: number;
};

export type PlanningMcPackageV1 = {
  schema_version: typeof PLANNING_MC_PACKAGE_SCHEMA_VERSION;
  planning_snapshot_id: string;
  planning_geometry_id: string;
  planning_extent: PlanningMcSnapshotExtent;
  source_layout_id?: string;
  source_geometry_id?: string;
  planning_summary: {
    radar_count: number;
    coverage_summary: PlanningMcPackageCoverageSummary;
    overlap_summary: PlanningMcPackageOverlapSummary;
    redundancy_summary: PlanningMcPackageRedundancySummary;
  };
  mc_preparation: {
    scenario_label: string;
    suggested_run_count: number;
    suggested_seed_base: number;
  };
  metadata: {
    created_utc: string;
    package_version: typeof PLANNING_MC_PACKAGE_VERSION;
  };
};

export type BuildPlanningMcPackageOptions = {
  scenarioLabel?: string;
  suggestedRunCount?: number;
  suggestedSeedBase?: number;
  createdUtc?: string;
};

function safePositiveInteger(value: number | undefined, fallback: number): number {
  if (value == null || !Number.isFinite(value)) return fallback;
  return Math.max(1, Math.floor(value));
}

function safeNonNegativeInteger(value: number | undefined, fallback: number): number {
  if (value == null || !Number.isFinite(value)) return fallback;
  return Math.max(0, Math.floor(value));
}

export function buildPlanningMcPackage(
  snapshot: PlanningMcSnapshotV1,
  options: BuildPlanningMcPackageOptions = {},
): PlanningMcPackageV1 {
  return {
    schema_version: PLANNING_MC_PACKAGE_SCHEMA_VERSION,
    planning_snapshot_id: snapshot.planning_snapshot_id,
    planning_geometry_id: snapshot.planning_geometry_id,
    planning_extent: snapshot.planning_extent,
    ...(snapshot.provenance.source_layout_id
      ? { source_layout_id: snapshot.provenance.source_layout_id }
      : {}),
    ...(snapshot.provenance.source_geometry_id
      ? { source_geometry_id: snapshot.provenance.source_geometry_id }
      : {}),
    planning_summary: {
      radar_count: snapshot.radars.radar_sites.length,
      coverage_summary: {
        coverage_percent: snapshot.analytics_summary.coverage_percent,
        blind_spot_summary: snapshot.analytics_summary.blind_spot_summary,
      },
      overlap_summary: {
        overlap_percent: snapshot.analytics_summary.overlap_percent,
      },
      redundancy_summary: {
        redundancy_percent: snapshot.analytics_summary.redundancy_percent,
      },
    },
    mc_preparation: {
      scenario_label: options.scenarioLabel ?? DEFAULT_PLANNING_MC_SCENARIO_LABEL,
      suggested_run_count: safePositiveInteger(
        options.suggestedRunCount,
        DEFAULT_PLANNING_MC_RUN_COUNT,
      ),
      suggested_seed_base: safeNonNegativeInteger(
        options.suggestedSeedBase,
        DEFAULT_PLANNING_MC_SEED_BASE,
      ),
    },
    metadata: {
      created_utc: options.createdUtc ?? snapshot.created_utc,
      package_version: PLANNING_MC_PACKAGE_VERSION,
    },
  };
}

export function planningPackageLinkId(pkg: PlanningMcPackageV1): string {
  return `rt_planning_package:${pkg.planning_snapshot_id}`;
}

export function exportPlanningMcPackageJson(pkg: PlanningMcPackageV1): string {
  if (pkg.schema_version !== PLANNING_MC_PACKAGE_SCHEMA_VERSION) {
    throw new Error(`expected schema_version ${PLANNING_MC_PACKAGE_SCHEMA_VERSION}`);
  }
  return exportJson(pkg);
}

export function suggestedPlanningMcPackageFilename(pkg: PlanningMcPackageV1): string {
  const safe = pkg.planning_snapshot_id.replace(/[^\w.-]+/g, "_").slice(0, 56);
  return `${safe || "rt_planning_mc"}_package.json`;
}

function triggerBrowserDownload(filename: string, json: string): void {
  const blob = new Blob([json], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

export function downloadPlanningMcPackage(pkg: PlanningMcPackageV1): void {
  triggerBrowserDownload(
    suggestedPlanningMcPackageFilename(pkg),
    exportPlanningMcPackageJson(pkg),
  );
}

export async function copyPlanningMcPackage(
  pkg: PlanningMcPackageV1,
): Promise<ClipboardResult> {
  return copyTextToClipboard(exportPlanningMcPackageJson(pkg));
}
