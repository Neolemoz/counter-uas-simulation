/** Live command path readiness — UI-only derived state (Web ↔ Gazebo Step 5). */

import { pickAdapterFields } from "@/adapter/adapterStatus";
import type { SessionRuntimeProfile } from "@/runtime/sessionRuntimeProfile";
import { isLiveRuntimeProfile } from "@/runtime/liveSessionUx";
import type { StatusBadgeTone } from "@/workstation/StatusBadge";

export type LiveCommandHealthState = "command_ready" | "command_unavailable";

export type LiveCommandHealthView = {
  state: LiveCommandHealthState;
  label: string;
  tone: StatusBadgeTone;
  detail: string;
};

const ENTITY_COMMAND_STATES = new Set(["running", "paused"]);

export const LIVE_COMMAND_READY_COPY =
  "Live command path ready — spawn/move/delete via bridge.";

export const LIVE_RUNTIME_CONNECTED_COPY =
  "Live Gazebo runtime connected (loopback).";

function stringOrNull(value: unknown): string | null {
  if (value === null || value === undefined) return null;
  return String(value);
}

export function isLiveCommandContext(
  requestedRuntimeProfile: SessionRuntimeProfile | null | undefined,
  sessionRuntimeProfile: string | null | undefined,
): boolean {
  return (
    isLiveRuntimeProfile(requestedRuntimeProfile) ||
    sessionRuntimeProfile === "live"
  );
}

export function deriveLiveCommandHealth({
  connected,
  requestedRuntimeProfile = null,
  sessionState,
  editingEnabled,
  sessionHealthPayload,
  livePreflightOk = null,
}: {
  connected: boolean;
  requestedRuntimeProfile?: SessionRuntimeProfile | null;
  sessionState: string;
  editingEnabled: boolean;
  sessionHealthPayload?: Record<string, unknown>;
  livePreflightOk?: boolean | null;
}): LiveCommandHealthView | null {
  const runtimeProfile = stringOrNull(sessionHealthPayload?.runtime_profile);
  if (!isLiveCommandContext(requestedRuntimeProfile, runtimeProfile)) {
    return null;
  }

  const { adapterAlive } = pickAdapterFields(sessionHealthPayload);
  const lifecycleOk = ENTITY_COMMAND_STATES.has(sessionState);

  const unavailable = (detail: string): LiveCommandHealthView => ({
    state: "command_unavailable",
    label: "Command path: unavailable",
    tone: "warn",
    detail,
  });

  const ready = (detail: string): LiveCommandHealthView => ({
    state: "command_ready",
    label: "Command path: ready",
    tone: "ok",
    detail,
  });

  if (!connected) {
    if (livePreflightOk === false) {
      return unavailable("Live preflight failed — start blocked.");
    }
    return unavailable("Connect a live session to enable the command path.");
  }

  if (adapterAlive === false) {
    return unavailable("Gazebo adapter not running — commands unavailable.");
  }

  if (!lifecycleOk) {
    return unavailable(
      `Session lifecycle ${sessionState} — entity commands blocked.`,
    );
  }

  if (!editingEnabled) {
    return unavailable(
      "Editing lock or inactive tab — entity commands blocked.",
    );
  }

  return ready(LIVE_COMMAND_READY_COPY);
}
