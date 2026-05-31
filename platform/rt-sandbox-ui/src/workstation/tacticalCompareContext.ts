import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import { tacticalStateFromSnapshot } from "@/hooks/useTacticalState";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import { entitiesFromSnapshot } from "@/telemetry/channelIndex";
import type { MirrorEntity } from "@/cesium/entityMarkers";

export type TacticalCompareSource = "embedded" | "session" | "previous";

export interface TacticalCompareContext {
  compareState: TacticalStatePayload;
  compareEntities: MirrorEntity[];
  source: TacticalCompareSource;
  compareSessionId?: string;
}

function parseEmbeddedCompareState(
  state: TacticalStatePayload | null | undefined,
): TacticalStatePayload | null {
  if (!state) return null;
  const raw = state as Record<string, unknown>;
  for (const key of [
    "compare_tactical_state",
    "previous_tactical_state",
    "baseline_tactical_state",
    "tactical_compare_state",
  ]) {
    const embedded = raw[key];
    if (embedded && typeof embedded === "object") {
      return embedded as TacticalStatePayload;
    }
  }
  return null;
}

function mirrorEntitiesFromSlot(slot: SessionSlot): MirrorEntity[] {
  return entitiesFromSnapshot(slot.snapshots.entity_pose_mirror)
    .map((row) => {
      const entityId = String(row.entity_id ?? "");
      const entityType = String(row.entity_type ?? "drone");
      const pose = row.pose;
      if (!entityId || !pose || typeof pose !== "object") return null;
      return {
        entity_id: entityId,
        entity_type: entityType,
        pose: pose as Record<string, unknown>,
      };
    })
    .filter((row): row is MirrorEntity => row != null);
}

export function resolveComparisonSessionId(
  activeSessionId: string | null,
  orderedSessionIds: readonly string[],
): string | null {
  if (!activeSessionId || orderedSessionIds.length < 2) return null;
  const other = orderedSessionIds.find((id) => id !== activeSessionId);
  return other ?? null;
}

export function resolveSessionCompareContext(
  activeSessionId: string | null,
  orderedSessionIds: readonly string[],
  slots: readonly SessionSlot[],
): TacticalCompareContext | null {
  const compareSessionId = resolveComparisonSessionId(
    activeSessionId,
    orderedSessionIds,
  );
  if (!compareSessionId) return null;
  const slot = slots.find((s) => s.sessionId === compareSessionId);
  if (!slot) return null;
  const compareState = tacticalStateFromSnapshot(slot.snapshots.tactical_state);
  if (!compareState) return null;
  const compareEntities = mirrorEntitiesFromSlot(slot);
  return {
    compareState,
    compareEntities,
    source: "session",
    compareSessionId,
  };
}

export function resolveTacticalCompareContext({
  currentState,
  previousState,
  activeSessionId,
  orderedSessionIds,
  slots,
  activeEntities,
}: {
  currentState: TacticalStatePayload | null | undefined;
  previousState?: TacticalStatePayload | null;
  activeSessionId: string | null;
  orderedSessionIds: readonly string[];
  slots: readonly SessionSlot[];
  activeEntities: MirrorEntity[];
}): TacticalCompareContext | null {
  const embedded = parseEmbeddedCompareState(currentState);
  if (embedded) {
    return {
      compareState: embedded,
      compareEntities: activeEntities,
      source: "embedded",
    };
  }

  const sessionCompare = resolveSessionCompareContext(
    activeSessionId,
    orderedSessionIds,
    slots,
  );
  if (sessionCompare) return sessionCompare;

  if (previousState) {
    return {
      compareState: previousState,
      compareEntities: activeEntities,
      source: "previous",
    };
  }

  return null;
}
