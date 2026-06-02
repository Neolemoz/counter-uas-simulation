import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import { resolveTacticalRoleIds } from "./tacticalTrajectoryLayer";
import type { TacticalCompareSource } from "@/workstation/tacticalCompareContext";
import { shortSessionId } from "@/workstation/sessionVisualIdentity";

export interface TacticalCompareDeltaLabels {
  timingLine: string | null;
  targetLine: string | null;
  assignmentLine: string | null;
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
    return {
      timingLine: null,
      targetLine: null,
      assignmentLine: null,
      block: null,
    };
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
  if (currentTarget && compareTarget && currentTarget !== compareTarget) {
    lines.push("target mismatch");
  }

  const currentInterceptor = currentRoles.interceptorId;
  const compareInterceptor = compareRoles.interceptorId;
  if (
    currentInterceptor &&
    compareInterceptor &&
    currentInterceptor !== compareInterceptor
  ) {
    lines.push("assignment mismatch");
  }

  const timingLine = lines.find((l) => l.startsWith("ΔTTI")) ?? null;
  const targetLine = lines.find((l) => l === "target mismatch") ?? null;
  const assignmentLine = lines.find((l) => l === "assignment mismatch") ?? null;
  const block = lines.length > 0 ? lines.join("\n") : null;

  return { timingLine, targetLine, assignmentLine, block };
}

export function formatTacticalCompareSummary(options: {
  source: TacticalCompareSource;
  compareSessionId?: string;
  deltas: TacticalCompareDeltaLabels;
  hasCompareGeometry: boolean;
}): string {
  const sourceLabel =
    options.source === "session" && options.compareSessionId
      ? `background ${shortSessionId(options.compareSessionId)}`
      : options.source === "embedded"
        ? "embedded compare state"
        : options.source === "previous"
          ? "prior active snapshot"
          : "compare source";

  const geometryNote = options.hasCompareGeometry
    ? "ghost path/solution on globe"
    : "no compare geometry on globe";

  if (options.deltas.block) {
    return `Tactical compare · ${sourceLabel} · ${options.deltas.block} · ${geometryNote} · display only`;
  }

  return `Tactical compare · ${sourceLabel} · roles aligned · ${geometryNote} · display only`;
}
