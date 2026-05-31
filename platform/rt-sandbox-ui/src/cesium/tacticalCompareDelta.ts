import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import { resolveTacticalRoleIds } from "./tacticalTrajectoryLayer";

export interface TacticalCompareDeltaLabels {
  timingLine: string | null;
  targetLine: string | null;
  block: string | null;
}

function finiteSeconds(value: unknown): number | null {
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

export function deriveTacticalCompareDeltaLabels(
  current: TacticalStatePayload | null | undefined,
  compare: TacticalStatePayload | null | undefined,
): TacticalCompareDeltaLabels {
  if (!current || !compare) {
    return { timingLine: null, targetLine: null, block: null };
  }

  const currentRoles = resolveTacticalRoleIds(current);
  const compareRoles = resolveTacticalRoleIds(compare);

  const lines: string[] = [];

  const currentTti = finiteSeconds(current.tti_s);
  const compareTti = finiteSeconds(compare.tti_s);
  if (currentTti != null && compareTti != null && Math.abs(currentTti - compareTti) >= 0.05) {
    const delta = currentTti - compareTti;
    const sign = delta >= 0 ? "+" : "";
    lines.push(`ΔTTI ${sign}${delta.toFixed(1)}s`);
  }

  const currentTarget = currentRoles.targetId;
  const compareTarget = compareRoles.targetId;
  if (
    currentTarget &&
    compareTarget &&
    currentTarget !== compareTarget
  ) {
    lines.push("target switched");
  }

  const timingLine = lines.find((l) => l.startsWith("ΔTTI")) ?? null;
  const targetLine = lines.find((l) => l === "target switched") ?? null;
  const block = lines.length > 0 ? lines.join("\n") : null;

  return { timingLine, targetLine, block };
}
