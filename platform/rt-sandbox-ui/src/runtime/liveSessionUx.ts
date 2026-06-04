/** Live Gazebo session UX copy and stop semantics (Web ↔ Gazebo Step 3). */

import type { SessionRuntimeProfile } from "@/runtime/sessionRuntimeProfile";

export const LIVE_STOP_GOVERNANCE =
  "Live stop terminates the Gazebo adapter.";

export const LIVE_MAINTAINER_SMOKE_HINT =
  "Maintainer validation: scripts/rt/rt_live_smoke.py against loopback bridge — not run from UI.";

export function isLiveRuntimeProfile(
  profile: SessionRuntimeProfile | null | undefined,
): boolean {
  return profile === "live";
}

export function liveStopCommandType(
  profile: SessionRuntimeProfile,
): "stop_sim" | "stop_session" {
  return profile === "live" ? "stop_sim" : "stop_session";
}
