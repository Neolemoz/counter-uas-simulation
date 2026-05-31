import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { EnuPoint } from "./tacticalTrajectoryLayer";

export const DEFAULT_INTERCEPTOR_SPEED_CAP_M_S = 25;

export interface TacticalTimingSeconds {
  ttiS: number | null;
  etaS: number | null;
}

function finitePositive(value: unknown): number | null {
  const n = Number(value);
  if (!Number.isFinite(n) || n < 0) return null;
  return n;
}

function readOptionalSeconds(
  state: TacticalStatePayload | null | undefined,
  key: "tti_s" | "eta_s",
): number | null {
  if (!state) return null;
  return finitePositive((state as Record<string, unknown>)[key]);
}

export function pathLengthM(points: EnuPoint[]): number {
  if (points.length < 2) return 0;
  let total = 0;
  for (let i = 1; i < points.length; i += 1) {
    const a = points[i - 1];
    const b = points[i];
    total += Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z);
  }
  return total;
}

export function interceptorSpeedCapMps(
  state: TacticalStatePayload | null | undefined,
): number {
  const cap = finitePositive(
    (state as Record<string, unknown> | null | undefined)
      ?.interceptor_speed_cap_m_s,
  );
  return cap && cap > 0 ? cap : DEFAULT_INTERCEPTOR_SPEED_CAP_M_S;
}

export function deriveTacticalTimingSeconds(
  state: TacticalStatePayload | null | undefined,
  pathPoints: EnuPoint[],
): TacticalTimingSeconds {
  const speedCap = interceptorSpeedCapMps(state);
  const pathS =
    pathPoints.length >= 2 ? pathLengthM(pathPoints) / speedCap : null;

  const ttiFromState = readOptionalSeconds(state, "tti_s");
  const etaFromState = readOptionalSeconds(state, "eta_s");

  return {
    ttiS: ttiFromState ?? pathS,
    etaS: etaFromState ?? pathS,
  };
}

export function formatTacticalTimingLine(
  prefix: "TTI" | "ETA",
  seconds: number | null,
): string | null {
  if (seconds == null) return null;
  return `${prefix} ${seconds.toFixed(1)}s`;
}

export function formatTacticalTimingBlock(
  timing: TacticalTimingSeconds,
): string | null {
  const tti = formatTacticalTimingLine("TTI", timing.ttiS);
  const eta = formatTacticalTimingLine("ETA", timing.etaS);
  const lines = [tti, eta].filter((line): line is string => line != null);
  return lines.length > 0 ? lines.join("\n") : null;
}
