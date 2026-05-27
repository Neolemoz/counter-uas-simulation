import {
  BACKGROUND_PULL_HZ,
  MAX_PULL_HZ,
} from "@/telemetry/constants";

export type SessionPollRole = "active" | "background";

export function formatLastPullAge(
  lastPullUtc: string | null,
  nowMs: number = Date.now(),
): string {
  if (!lastPullUtc) return "never";
  const parsed = Date.parse(lastPullUtc);
  if (Number.isNaN(parsed)) return "unknown";
  const ageSec = Math.max(0, Math.floor((nowMs - parsed) / 1000));
  if (ageSec < 60) return `${ageSec}s ago`;
  const ageMin = Math.floor(ageSec / 60);
  if (ageMin < 60) return `${ageMin}m ago`;
  return `${Math.floor(ageMin / 60)}h ago`;
}

export function expectedPullIntervalMs(
  role: SessionPollRole,
  pullHz: number = 1,
): number {
  const hz =
    role === "active"
      ? Math.min(Math.max(pullHz, 0.1), MAX_PULL_HZ)
      : BACKGROUND_PULL_HZ;
  return Math.max(100, Math.floor(1000 / hz));
}

/** Stale when last pull age exceeds ~2× expected interval for role. */
export function isPullAgeStale(
  role: SessionPollRole,
  lastPullUtc: string | null,
  pullHz: number = 1,
  nowMs: number = Date.now(),
): boolean {
  if (!lastPullUtc) return true;
  const parsed = Date.parse(lastPullUtc);
  if (Number.isNaN(parsed)) return true;
  const ageMs = nowMs - parsed;
  return ageMs > expectedPullIntervalMs(role, pullHz) * 2;
}
