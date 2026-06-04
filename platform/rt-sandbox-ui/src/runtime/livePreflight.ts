/** Live Gazebo host preflight mirror (rt_live_runtime_preflight_v1). */

export type LivePreflightChecks = {
  ros2_available: boolean;
  gz_available: boolean;
  rt_sandbox_gz_available: boolean;
};

export type LivePreflightResult = {
  schema?: string;
  ok: boolean;
  checks: LivePreflightChecks;
  blockers: string[];
  message?: string;
};

export type LivePreflightRow = {
  id: keyof LivePreflightChecks;
  label: string;
  ok: boolean;
};

const CHECK_LABELS: Record<keyof LivePreflightChecks, string> = {
  ros2_available: "ros2 on PATH",
  gz_available: "gz (Gazebo Sim) on PATH",
  rt_sandbox_gz_available: "rt_sandbox_gz ROS package",
};

export function parseLivePreflight(value: unknown): LivePreflightResult | null {
  if (!value || typeof value !== "object") return null;
  const raw = value as Record<string, unknown>;
  const checksRaw = raw.checks;
  if (!checksRaw || typeof checksRaw !== "object") return null;
  const checksIn = checksRaw as Record<string, unknown>;
  const checks: LivePreflightChecks = {
    ros2_available: checksIn.ros2_available === true,
    gz_available: checksIn.gz_available === true,
    rt_sandbox_gz_available: checksIn.rt_sandbox_gz_available === true,
  };
  const blockers = Array.isArray(raw.blockers)
    ? raw.blockers.filter((item): item is string => typeof item === "string")
    : [];
  return {
    schema: typeof raw.schema === "string" ? raw.schema : undefined,
    ok: raw.ok === true,
    checks,
    blockers,
    message: typeof raw.message === "string" ? raw.message : undefined,
  };
}

export function livePreflightRows(result: LivePreflightResult | null): LivePreflightRow[] {
  if (!result) {
    return (Object.keys(CHECK_LABELS) as Array<keyof LivePreflightChecks>).map((id) => ({
      id,
      label: CHECK_LABELS[id],
      ok: false,
    }));
  }
  return (Object.keys(CHECK_LABELS) as Array<keyof LivePreflightChecks>).map((id) => ({
    id,
    label: CHECK_LABELS[id],
    ok: result.checks[id],
  }));
}

export function livePreflightSummary(result: LivePreflightResult | null): string {
  if (!result) return "Preflight not checked yet.";
  if (result.ok) return "Live runtime preflight passed.";
  if (result.blockers.length > 0) return result.blockers.join("; ");
  return result.message ?? "Live runtime unavailable.";
}
