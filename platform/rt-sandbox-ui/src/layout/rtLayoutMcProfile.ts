/**
 * Browser-side mirror of scripts/evaluation/rt_layout_mc_profile.py (dry-run preview only).
 * Keep behavior aligned with the Python translator; do not invoke Monte Carlo from the UI.
 */

import { sha256Hex16 } from "@/experiment/sha256Hex";

export const LAYOUT_SCHEMA_VERSION = "rt_layout_scenario_v1" as const;
export const PROFILE_SCHEMA_VERSION = "rt_layout_mc_profile_v1" as const;

const ENTITY_TYPE_SET = new Set<string>([
  "radar",
  "interceptor",
  "drone",
  "waypoint_marker",
]);

const REQUIRED_LAYOUT_KEYS = [
  "schema_version",
  "layout_id",
  "terrain_preset",
  "entities",
  "source",
] as const;

export type RtLayoutPose = {
  x: number;
  y: number;
  z: number;
  yaw_deg?: number;
};

export type RtLayoutEntity = {
  entity_type: string;
  pose: RtLayoutPose;
};

export type RtLayoutScenarioV1 = {
  schema_version: typeof LAYOUT_SCHEMA_VERSION;
  layout_id: string;
  terrain_preset: string;
  entities: RtLayoutEntity[];
  source: Record<string, unknown>;
  created_utc?: string;
  notes?: string;
};

export type LayoutValidation = {
  ok: boolean;
  issues: string[];
  warnings: string[];
};

export type McProfilePreview = {
  schema_version: typeof PROFILE_SCHEMA_VERSION;
  source_layout_id: string;
  geometry_id: string;
  scenario_suggestion: string;
  launch_args: string;
  launch_args_fields: Record<string, string>;
  entity_counts: Record<string, number>;
  warnings: string[];
  unsupported_fields: Array<{ field: string; reason: string }>;
  source: {
    artifact_schema: string;
    layout_source: unknown;
  };
  generated_utc: string;
};

function floatish(value: unknown): boolean {
  if (typeof value === "number" && Number.isFinite(value)) return true;
  if (typeof value === "string" && value.trim() !== "") {
    const n = Number(value);
    return Number.isFinite(n);
  }
  return false;
}

function normalizedPose(pose: Record<string, unknown>): RtLayoutPose {
  const out: RtLayoutPose = {
    x: Number(pose.x),
    y: Number(pose.y),
    z: Number(pose.z),
  };
  if (pose.yaw_deg !== undefined && pose.yaw_deg !== null) {
    out.yaw_deg = Number(pose.yaw_deg);
  }
  return out;
}

export function normalizedEntities(data: RtLayoutScenarioV1 | Record<string, unknown>): RtLayoutEntity[] {
  const entities: RtLayoutEntity[] = [];
  const raw = data.entities;
  if (!Array.isArray(raw)) return entities;
  for (const entity of raw) {
    if (!entity || typeof entity !== "object") continue;
    const row = entity as Record<string, unknown>;
    const pose = row.pose;
    if (!pose || typeof pose !== "object") continue;
    entities.push({
      entity_type: String(row.entity_type ?? ""),
      pose: normalizedPose(pose as Record<string, unknown>),
    });
  }
  return entities;
}

export function validateLayout(data: Record<string, unknown>): LayoutValidation {
  const issues: string[] = [];
  const warnings: string[] = [];

  for (const key of REQUIRED_LAYOUT_KEYS) {
    if (!(key in data)) {
      issues.push(`missing required field: ${key}`);
    }
  }

  if (data.schema_version !== LAYOUT_SCHEMA_VERSION) {
    issues.push(`schema_version must be '${LAYOUT_SCHEMA_VERSION}'`);
  }
  if (typeof data.layout_id !== "string" || !String(data.layout_id).trim()) {
    issues.push("layout_id must be a non-empty string");
  }
  if (typeof data.terrain_preset !== "string" || !String(data.terrain_preset).trim()) {
    issues.push("terrain_preset must be a non-empty string");
  }
  if (!data.source || typeof data.source !== "object") {
    issues.push("source must be an object");
  }

  const entities = data.entities;
  if (!Array.isArray(entities)) {
    issues.push("entities must be a list");
  } else if (entities.length === 0) {
    warnings.push("layout has no entities");
  } else {
    entities.forEach((entity, idx) => {
      if (!entity || typeof entity !== "object") {
        issues.push(`entities[${idx}] must be an object`);
        return;
      }
      const row = entity as Record<string, unknown>;
      const entityType = row.entity_type;
      if (!ENTITY_TYPE_SET.has(String(entityType))) {
        issues.push(`entities[${idx}].entity_type unsupported: ${String(entityType)}`);
      }
      const pose = row.pose;
      if (!pose || typeof pose !== "object") {
        issues.push(`entities[${idx}].pose must be an object`);
        return;
      }
      const poseRow = pose as Record<string, unknown>;
      for (const key of ["x", "y", "z"] as const) {
        if (!(key in poseRow)) {
          issues.push(`entities[${idx}].pose missing ${key}`);
        } else if (!floatish(poseRow[key])) {
          issues.push(`entities[${idx}].pose.${key} must be numeric`);
        }
      }
      if ("yaw_deg" in poseRow && !floatish(poseRow.yaw_deg)) {
        issues.push(`entities[${idx}].pose.yaw_deg must be numeric when present`);
      }
    });
  }

  return { ok: issues.length === 0, issues, warnings };
}

function stableStringify(value: unknown): string {
  if (value === null || typeof value !== "object") {
    return JSON.stringify(value);
  }
  if (Array.isArray(value)) {
    return `[${value.map((item) => stableStringify(item)).join(",")}]`;
  }
  const obj = value as Record<string, unknown>;
  const keys = Object.keys(obj).sort();
  return `{${keys.map((key) => `${JSON.stringify(key)}:${stableStringify(obj[key])}`).join(",")}}`;
}

export function geometryFingerprint(data: RtLayoutScenarioV1 | Record<string, unknown>): string {
  const canonical = {
    schema_version: LAYOUT_SCHEMA_VERSION,
    terrain_preset: String(data.terrain_preset ?? ""),
    entities: normalizedEntities(data),
  };
  const digest = sha256Hex16(stableStringify(canonical));
  return `rt_layout:sha256:${digest}`;
}

/** Match Python f"{value:.6g}". */
export function fmtLaunchFloat(value: number): string {
  const s = Number(value).toPrecision(6);
  if (s.includes("e") || s.includes("E")) return s;
  return String(Number.parseFloat(s));
}

function countEntities(entities: RtLayoutEntity[]): Record<string, number> {
  const counts: Record<string, number> = {
    drone: 0,
    interceptor: 0,
    radar: 0,
    waypoint_marker: 0,
  };
  for (const entity of entities) {
    const key = entity.entity_type;
    if (key in counts) counts[key] += 1;
  }
  return counts;
}

export function translateLayoutToProfile(data: RtLayoutScenarioV1): McProfilePreview {
  const validation = validateLayout(data as unknown as Record<string, unknown>);
  if (!validation.ok) {
    throw new Error(validation.issues.join("; "));
  }

  const entities = normalizedEntities(data);
  const counts = countEntities(entities);
  const drones = entities.filter((e) => e.entity_type === "drone");
  const interceptors = entities.filter((e) => e.entity_type === "interceptor");
  const radars = entities.filter((e) => e.entity_type === "radar");
  const waypoints = entities.filter((e) => e.entity_type === "waypoint_marker");

  const warnings = [...validation.warnings];
  const unsupported_fields: Array<{ field: string; reason: string }> = [];
  const launch_fields: Record<string, string> = {};

  if (drones.length === 1) {
    const pose = drones[0].pose;
    launch_fields.target_start_x_m = fmtLaunchFloat(pose.x);
    launch_fields.target_start_y_m = fmtLaunchFloat(pose.y);
    launch_fields.target_start_z_m = fmtLaunchFloat(pose.z);
  } else if (drones.length === 0) {
    warnings.push("no drone entity: target_start_* launch args were not generated");
  } else {
    warnings.push(
      "multiple drone entities: MC target_start_* mapping supports exactly one drone today",
    );
    unsupported_fields.push({
      field: "entities[drone]",
      reason: "multiple drone layout is not launch-arg compatible today",
    });
  }

  if (interceptors.length === 3) {
    const coords: string[] = [];
    for (const entity of interceptors) {
      const pose = entity.pose;
      coords.push(fmtLaunchFloat(pose.x), fmtLaunchFloat(pose.y));
    }
    launch_fields.interceptor_ic_layout = `custom:${coords.join(",")}`;
  } else if (interceptors.length === 0) {
    warnings.push("no interceptor entities: interceptor_ic_layout was not generated");
  } else {
    warnings.push(
      "interceptor_ic_layout custom mapping requires exactly three interceptors today",
    );
    unsupported_fields.push({
      field: "entities[interceptor]",
      reason: `found ${interceptors.length} interceptor entities; expected 3`,
    });
  }

  for (const [entityType, rows] of [
    ["radar", radars],
    ["waypoint_marker", waypoints],
  ] as const) {
    if (rows.length > 0) {
      warnings.push(`${entityType} entities are retained as metadata only for MC preview`);
      unsupported_fields.push({
        field: `entities[${entityType}]`,
        reason: "no current Monte Carlo launch-arg mapping",
      });
    }
  }

  const scenario_suggestion = drones.length > 1 ? "multi" : "single";
  const launch_args = Object.entries(launch_fields)
    .map(([key, value]) => `${key}:=${value}`)
    .join(" ");

  return {
    schema_version: PROFILE_SCHEMA_VERSION,
    source_layout_id: data.layout_id,
    geometry_id: geometryFingerprint(data),
    scenario_suggestion,
    launch_args,
    launch_args_fields: launch_fields,
    entity_counts: counts,
    warnings,
    unsupported_fields,
    source: {
      artifact_schema: LAYOUT_SCHEMA_VERSION,
      layout_source: data.source,
    },
    generated_utc: new Date().toISOString().replace(/\.\d{3}Z$/, "Z"),
  };
}
