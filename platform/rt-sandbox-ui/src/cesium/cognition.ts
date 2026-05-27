import { FORBIDDEN_LEXICON } from "@/governance/banners";
import { cognitionSummary } from "@/telemetry/cognition";
import { CESIUM_SCENARIO_CAVEAT } from "./constants";

export function cesiumViewSummary(options: {
  pendingReconcile: boolean;
  entityCount: number;
  editingEnabled: boolean;
}): {
  caveat: string;
  mirrorNote: string;
  dualSurfaceNote: string;
  reconcileNote?: string;
  entityCountLabel: string;
} {
  return {
    caveat: CESIUM_SCENARIO_CAVEAT,
    mirrorNote: options.editingEnabled
      ? "Click globe to spawn; drag markers to move — registry commands authoritative."
      : "Editing blocked for current lifecycle state.",
    dualSurfaceNote:
      "SVG grid and Cesium globe both edit this session via spawn_entity / move_entity / delete_entity.",
    reconcileNote: options.pendingReconcile
      ? "Awaiting pull reconcile after registry command."
      : undefined,
    entityCountLabel: `${options.entityCount} marker(s) on globe`,
  };
}

export function cesiumEditingAuthorityNote(): string {
  return "Registry command truth — entity_pose_mirror pull is explanatory telemetry only; not replay authority.";
}

export function cesiumCognitionFromPayload(payload: Record<string, unknown>) {
  return cognitionSummary(payload);
}

export function containsForbiddenLexicon(text: string): boolean {
  const lower = text.toLowerCase();
  return FORBIDDEN_LEXICON.some((term) => lower.includes(term.toLowerCase()));
}

export function markerStyleForHealth(
  syncHealth: string | undefined,
  telemetryHealth: string | undefined,
  entityDriftM?: number | null,
  driftThresholdM = 2.0,
): "ok" | "stale" | "warn" {
  if (telemetryHealth === "stale" || telemetryHealth === "feedback_lost") {
    return "stale";
  }
  if (typeof entityDriftM === "number" && entityDriftM > driftThresholdM) {
    return "warn";
  }
  if (syncHealth && syncHealth !== "ok") {
    return "warn";
  }
  return "ok";
}

export function cesiumSyncAdapterNote(
  worldSummary: Record<string, unknown> | undefined,
): string {
  const mode = worldSummary?.adapter_mode;
  if (mode === "live") {
    return "Live Gazebo — entity_state is sim truth; registry remains command authority.";
  }
  if (mode === "mock") {
    return "Mock adapter — in-memory sim feedback; enable live mode for Gazebo truth.";
  }
  return "Adapter off — pose sync N/A; mirrors reflect stub path only.";
}
