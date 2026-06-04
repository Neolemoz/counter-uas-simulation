/** Frozen PLAT-RT-S4 telemetry channels. */

export const TELEMETRY_CHANNELS = [
  "session_health",
  "lifecycle_state",
  "world_summary",
  "entity_pose_mirror",
  "clock_mirror",
  "tactical_state",
  "tactical_recommendation",
  "intelligence_advisory",
] as const;

export type TelemetryChannel = (typeof TELEMETRY_CHANNELS)[number];

export const DIAGNOSTIC_TELEMETRY_CHANNELS = [
  "session_health",
  "lifecycle_state",
  "world_summary",
] as const;

/** Governance cap: telemetry_update_rate_cap_hz */
export const MAX_PULL_HZ = 10;

export const BACKGROUND_PULL_HZ = 1;

export const DEFAULT_PULL_HZ = 1;

export const BRIDGE_COMMAND_URL = "/v1/command";
export const BRIDGE_PULL_URL = "/v1/telemetry/pull";
